#pragma once

#include "localization/likelihood_field.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <vector>
#include <random>

struct ParticleFilterParameters {
    int num_particles = 500;
    unsigned int random_seed = 42;
    double likelihood_max_distance = 1.0;
    std::size_t scan_beam_step = 10;
    // Beam endpoint measurement model (likelihood-field, Thrun "Probabilistic
    // Robotics"). The per-beam probability is z_hit * q + z_rand, where q is the
    // hit probability exp(-d^2 / 2*sigma_hit^2) baked into the likelihood field
    // at build time. A particle's weight is the PRODUCT over beams (accumulated
    // as a sum of logs). z_rand floors each beam so one stray reading cannot
    // zero out an otherwise-good particle. sigma_hit is consumed by the field
    // build (buildLikelihoodField); z_hit/z_rand are applied in the scorer.
    double measurement_sigma_hit = 0.2;
    double measurement_z_hit = 0.95;
    double measurement_z_rand = 0.05;
    double translation_noise_from_translation = 0.02;
    double translation_noise_base = 0.005;
    double rotation_noise_from_rotation = 0.05;
    double rotation_noise_from_translation = 0.01;
    double rotation_noise_base = 0.002;
    // Minimum translation (m) for the heading-of-travel (delta_rot1) to be
    // trustworthy. Below this the move is treated as pure rotation, because
    // atan2() on near-zero (noisy) translation yields a meaningless direction
    // that would scatter the particles during in-place spins.
    double min_translation_for_heading = 0.01;
    double resample_xy_noise_std = 0.02;
    double resample_theta_noise_std = 0.03;
    // Recovery injection: a linear ramp on the "confident" scan fit (the mean
    // fit of the best-matching ~20% of particles). The injected fraction is
    //   clamp((score_high - confident) / (score_high - score_low), 0, 1) * max
    // so at/above score_high it injects nothing and at/below score_low it
    // injects max_fraction. The signal is the confident fit (not the
    // all-particle average) on purpose: random/lost particles never enter the
    // top cluster, so injecting them cannot drag the signal down and lock the
    // filter into permanent injection.
    double recovery_score_high = 0.95;
    double recovery_score_low = 0.50;
    double recovery_max_fraction = 0.5;
};

struct Particle {
    double x;
    double y;
    double theta;
    double weight;
};

struct ScanScoreStats {
    double best_score;
    double worst_score;
    double average_score;
    // Mean fit of the best-matching ~20% of particles. Unlike average_score this
    // is robust to injected/lost particles, so it drives recovery injection.
    double confident_score;
};

struct EstimatedPose {
    double x;
    double y;
    double theta;
};

class ParticleFilter
{
public:
    explicit ParticleFilter(
        const ParticleFilterParameters & parameters = ParticleFilterParameters());

    void configure(const ParticleFilterParameters & parameters);

    void initializeUniform(const nav_msgs::msg::OccupancyGrid & map);

    // Seed all particles from a Gaussian around a known pose (e.g. RViz
    // "2D Pose Estimate"). std_xy / std_theta are the 1-sigma spread of the
    // initial-pose uncertainty.
    void initializeGaussian(double x, double y, double theta,
                            double std_xy, double std_theta);

    void buildLikelihoodField(const nav_msgs::msg::OccupancyGrid & map);

    void sampleMotionModel(double old_x, double old_y, double old_theta,
                            double new_x, double new_y, double new_theta);

    ScanScoreStats scoreParticlesWithScan(const sensor_msgs::msg::LaserScan& scan);

    void resample();

    EstimatedPose estimatePose() const;

    const nav_msgs::msg::OccupancyGrid & likelihoodFieldMap() const;
    
    const std::vector<Particle> & particles() const;

private:
    struct FreeCell {
        int col;
        int row;
    };

    void rememberFreeCells(const nav_msgs::msg::OccupancyGrid & map);
    Particle sampleRandomFreeParticle();
    // Update the recovery-injection fraction from the confident scan fit of the
    // latest measurement update (linear ramp, see ParticleFilterParameters).
    void updateRecoveryFraction(double confident_fit);
    // Recompute the cached pose estimate from the current (freshly scored)
    // particle weights. Must be called while the weights are meaningful, i.e.
    // after scoring and before resampling resets them to uniform.
    void computeWeightedEstimate();

    ParticleFilterParameters parameters_;
    std::vector<Particle> particles_;
    std::vector<FreeCell> free_cells_;
    LikelihoodField likelihood_field_;
    std::default_random_engine rng_;
    double map_resolution_ = 0.0;
    double map_origin_x_ = 0.0;
    double map_origin_y_ = 0.0;
    double current_recovery_particle_fraction_ = 0.0;
    // Cached pose estimate. Recomputed from fresh weights at each scan update
    // and carried forward by the motion model between scans, so estimatePose()
    // is a cheap, consistent read that does not depend on post-resample weights.
    EstimatedPose last_estimate_{0.0, 0.0, 0.0};
};
