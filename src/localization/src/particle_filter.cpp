#include "localization/particle_filter.hpp"
#include "localization/geometry_utils.hpp"

#include <algorithm>
#include <limits>
#include <random>
#include <cmath>

ParticleFilter::ParticleFilter(const ParticleFilterParameters & parameters)
{
    configure(parameters);
}

void ParticleFilter::configure(const ParticleFilterParameters & parameters)
{
    parameters_ = parameters;
    rng_.seed(parameters_.random_seed);
    particles_.resize(parameters_.num_particles);
    current_recovery_particle_fraction_ = parameters_.recovery_fraction_high;
}

void ParticleFilter::initializeUniform(const nav_msgs::msg::OccupancyGrid & map)
{
    rememberFreeCells(map);

    if (free_cells_.empty()) {
        return;
    }

    for (auto & p : particles_) {
        p = sampleRandomFreeParticle();
    }
}

void ParticleFilter::initializeGaussian(
    double x, double y, double theta, double std_xy, double std_theta)
{
    std::normal_distribution<double> xy_noise(0.0, std_xy);
    std::normal_distribution<double> theta_noise(0.0, std_theta);

    const double uniform_weight = 1.0 / std::max(1, parameters_.num_particles);
    for (auto & p : particles_) {
        p.x = x + xy_noise(rng_);
        p.y = y + xy_noise(rng_);
        p.theta = normalizeAngle(theta + theta_noise(rng_));
        p.weight = uniform_weight;
    }
}

void ParticleFilter::buildLikelihoodField(const nav_msgs::msg::OccupancyGrid & map)
{
    likelihood_field_.build(map, parameters_.likelihood_max_distance);
}

void ParticleFilter::sampleMotionModel(
    double old_x, double old_y, double old_theta,
    double new_x, double new_y, double new_theta)
{
    // Decompose odometry into 3 primitive moves:
    // 1. rotate to face the direction of travel
    // 2. translate forward
    // 3. rotate to final heading
    double delta_rot1 = 0.0;
    double delta_trans = std::sqrt(std::pow(new_x - old_x, 2) + std::pow(new_y - old_y, 2));
    if (delta_trans > 1e-6) {
        delta_rot1 = normalizeAngle(std::atan2(new_y - old_y, new_x - old_x) - old_theta);
    }
    double delta_rot2 = normalizeAngle(new_theta - old_theta - delta_rot1);

    double trans_noise_std =
        parameters_.translation_noise_from_translation * delta_trans +
        parameters_.translation_noise_base;
    double rot1_noise_std =
        parameters_.rotation_noise_from_rotation * std::abs(delta_rot1) +
        parameters_.rotation_noise_from_translation * delta_trans +
        parameters_.rotation_noise_base;
    double rot2_noise_std =
        parameters_.rotation_noise_from_rotation * std::abs(delta_rot2) +
        parameters_.rotation_noise_from_translation * delta_trans +
        parameters_.rotation_noise_base;

    std::normal_distribution<double> trans_noise(0.0, trans_noise_std);
    std::normal_distribution<double> rot1_noise(0.0, rot1_noise_std);
    std::normal_distribution<double> rot2_noise(0.0, rot2_noise_std);

    for (auto & p : particles_) {
        double noisy_rot1 = delta_rot1 + rot1_noise(rng_);
        double noisy_trans = delta_trans + trans_noise(rng_);
        double noisy_rot2 = delta_rot2 + rot2_noise(rng_);

        p.x     += noisy_trans * std::cos(p.theta + noisy_rot1);
        p.y     += noisy_trans * std::sin(p.theta + noisy_rot1);
        p.theta = normalizeAngle(p.theta + noisy_rot1 + noisy_rot2);
    }
}

ScanScoreStats ParticleFilter::scoreParticlesWithScan(const sensor_msgs::msg::LaserScan & scan)
{
    ScanScoreStats stats;
    stats.best_score = 0.0;
    stats.worst_score = std::numeric_limits<double>::max();
    stats.average_score = 0.0;

    if (!likelihood_field_.hasMap()) {
        return stats;
    }

    double score_sum = 0.0;

    for (auto & p : particles_) {
        double particle_score = 0.0;
        int used_beams = 0;

        for (size_t i = 0; i < scan.ranges.size(); i += parameters_.scan_beam_step) {
            float range = scan.ranges[i];

            if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
                continue;
            }

            double beam_angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
            double hit_x = p.x + range * std::cos(p.theta + beam_angle);
            double hit_y = p.y + range * std::sin(p.theta + beam_angle);

            particle_score += likelihood_field_.valueAtWorld(hit_x, hit_y);
            used_beams++;
        }

        if (used_beams > 0) {
            particle_score /= used_beams;
        }

        p.weight = particle_score;
        score_sum += particle_score;
        stats.best_score = std::max(stats.best_score, particle_score);
        stats.worst_score = std::min(stats.worst_score, particle_score);
    }

    if (stats.worst_score == std::numeric_limits<double>::max()) {
        stats.worst_score = 0.0;
    }

    stats.average_score = score_sum / particles_.size();
    updateRecoveryFraction(stats.best_score);

    return stats;
}

void ParticleFilter::resample()
{
    double weight_sum = 0.0;
    for (const auto & p : particles_) {
        weight_sum += p.weight;
    }

    if (weight_sum <= 0.0 || !std::isfinite(weight_sum)) {
        double equal_weight = 1.0 / parameters_.num_particles;
        for (auto & p : particles_) {
            p.weight = equal_weight;
        }
        weight_sum = 1.0;

        if (free_cells_.empty()) {
            return;
        }
    }

    std::vector<double> cumulative_weights;
    cumulative_weights.reserve(particles_.size());
    double cumulative_sum = 0.0;

    for (auto & p : particles_) {
        p.weight /= weight_sum;
        cumulative_sum += p.weight;
        cumulative_weights.push_back(cumulative_sum);
    }

    // Avoid tiny floating-point error, so the final pointer can always find a particle.
    cumulative_weights.back() = 1.0;

    std::normal_distribution<double> xy_noise(0.0, parameters_.resample_xy_noise_std);
    std::normal_distribution<double> theta_noise(0.0, parameters_.resample_theta_noise_std);
    std::uniform_real_distribution<double> start_distribution(
        0.0, 1.0 / parameters_.num_particles);
    std::uniform_real_distribution<double> recovery_distribution(0.0, 1.0);

    std::vector<Particle> new_particles;
    new_particles.reserve(parameters_.num_particles);

    double pointer = start_distribution(rng_);
    double step = 1.0 / parameters_.num_particles;
    size_t particle_index = 0;

    for (int i = 0; i < parameters_.num_particles; ++i) {
        if (!free_cells_.empty() &&
            recovery_distribution(rng_) < current_recovery_particle_fraction_) {
            new_particles.push_back(sampleRandomFreeParticle());
            pointer += step;
            continue;
        }

        while (pointer > cumulative_weights[particle_index]) {
            particle_index++;
        }

        Particle copied = particles_[particle_index];

        copied.x += xy_noise(rng_);
        copied.y += xy_noise(rng_);
        copied.theta = normalizeAngle(copied.theta + theta_noise(rng_));

        new_particles.push_back(copied);
        pointer += step;
    }

    particles_ = new_particles;
}

void ParticleFilter::rememberFreeCells(const nav_msgs::msg::OccupancyGrid & map)
{
    map_resolution_ = map.info.resolution;
    map_origin_x_ = map.info.origin.position.x;
    map_origin_y_ = map.info.origin.position.y;
    free_cells_.clear();

    const int width = map.info.width;
    const int height = map.info.height;
    free_cells_.reserve(map.data.size());

    for (int row = 0; row < height; ++row) {
        for (int col = 0; col < width; ++col) {
            const int index = row * width + col;
            if (map.data[index] == 0) {
                free_cells_.push_back({col, row});
            }
        }
    }
}

Particle ParticleFilter::sampleRandomFreeParticle()
{
    std::uniform_int_distribution<std::size_t> rand_cell(0, free_cells_.size() - 1);
    std::uniform_real_distribution<double> rand_theta(-M_PI, M_PI);

    const FreeCell & cell = free_cells_[rand_cell(rng_)];

    Particle particle;
    particle.x = map_origin_x_ + (cell.col + 0.5) * map_resolution_;
    particle.y = map_origin_y_ + (cell.row + 0.5) * map_resolution_;
    particle.theta = rand_theta(rng_);
    particle.weight = 1.0 / parameters_.num_particles;
    return particle;
}

void ParticleFilter::updateRecoveryFraction(double best_score)
{
    if (!std::isfinite(best_score)) {
        current_recovery_particle_fraction_ = parameters_.recovery_fraction_min;
        return;
    }

    if (best_score >= parameters_.recovery_score_high) {
        current_recovery_particle_fraction_ = parameters_.recovery_fraction_high;
    } else if (best_score >= parameters_.recovery_score_medium) {
        const double ratio =
            (parameters_.recovery_score_high - best_score) /
            (parameters_.recovery_score_high - parameters_.recovery_score_medium);
        current_recovery_particle_fraction_ =
            parameters_.recovery_fraction_high +
            ratio * (parameters_.recovery_fraction_medium - parameters_.recovery_fraction_high);
    } else if (best_score >= parameters_.recovery_score_low) {
        const double ratio =
            (parameters_.recovery_score_medium - best_score) /
            (parameters_.recovery_score_medium - parameters_.recovery_score_low);
        current_recovery_particle_fraction_ =
            parameters_.recovery_fraction_medium +
            ratio * (parameters_.recovery_fraction_low - parameters_.recovery_fraction_medium);
    } else if (best_score >= parameters_.recovery_score_min) {
        const double ratio =
            (parameters_.recovery_score_low - best_score) /
            (parameters_.recovery_score_low - parameters_.recovery_score_min);
        current_recovery_particle_fraction_ =
            parameters_.recovery_fraction_low +
            ratio * (parameters_.recovery_fraction_min - parameters_.recovery_fraction_low);
    } else {
        current_recovery_particle_fraction_ = parameters_.recovery_fraction_min;
    }

    current_recovery_particle_fraction_ =
        std::clamp(current_recovery_particle_fraction_, 0.0, 1.0);
}

EstimatedPose ParticleFilter::estimatePose() const
{
    EstimatedPose pose;
    pose.x = 0.0;
    pose.y = 0.0;
    pose.theta = 0.0;

    if (particles_.empty()) {
        return pose;
    }

    std::vector<Particle> best_particles = particles_;
    std::sort(
        best_particles.begin(),
        best_particles.end(),
        [](const Particle & a, const Particle & b) {
            return a.weight > b.weight;
        });

    const std::size_t particles_to_average = std::max<std::size_t>(
        1,
        static_cast<std::size_t>(std::ceil(best_particles.size() * 0.20)));

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_sin = 0.0;
    double sum_cos = 0.0;
    double sum_weight = 0.0;

    for (std::size_t i = 0; i < particles_to_average; ++i) {
        const Particle & p = best_particles[i];
        sum_x += p.x * p.weight;
        sum_y += p.y * p.weight;
        sum_sin += std::sin(p.theta) * p.weight;
        sum_cos += std::cos(p.theta) * p.weight;
        sum_weight += p.weight;
    }

    if (sum_weight <= 0.0 || !std::isfinite(sum_weight)) {
        sum_x = 0.0;
        sum_y = 0.0;
        sum_sin = 0.0;
        sum_cos = 0.0;

        for (std::size_t i = 0; i < particles_to_average; ++i) {
            const Particle & p = best_particles[i];
            sum_x += p.x;
            sum_y += p.y;
            sum_sin += std::sin(p.theta);
            sum_cos += std::cos(p.theta);
        }

        sum_weight = static_cast<double>(particles_to_average);
    }

    pose.x = sum_x / sum_weight;
    pose.y = sum_y / sum_weight;
    pose.theta = std::atan2(sum_sin, sum_cos);

    return pose;
}

const std::vector<Particle> & ParticleFilter::particles() const
{
    return particles_;
}

const nav_msgs::msg::OccupancyGrid & ParticleFilter::likelihoodFieldMap() const
{
    return likelihood_field_.message();
}
