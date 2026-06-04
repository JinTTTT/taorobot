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
    double translation_noise_from_translation = 0.02;
    double translation_noise_base = 0.005;
    double rotation_noise_from_rotation = 0.05;
    double rotation_noise_from_translation = 0.01;
    double rotation_noise_base = 0.002;
    double resample_xy_noise_std = 0.02;
    double resample_theta_noise_std = 0.03;
    double recovery_score_high = 0.99;
    double recovery_score_medium = 0.90;
    double recovery_score_low = 0.80;
    double recovery_score_min = 0.70;
    double recovery_fraction_high = 0.0;
    double recovery_fraction_medium = 0.10;
    double recovery_fraction_low = 0.30;
    double recovery_fraction_min = 0.50;
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
    void updateRecoveryFraction(double best_score);

    ParticleFilterParameters parameters_;
    std::vector<Particle> particles_;
    std::vector<FreeCell> free_cells_;
    LikelihoodField likelihood_field_;
    std::default_random_engine rng_;
    double map_resolution_ = 0.0;
    double map_origin_x_ = 0.0;
    double map_origin_y_ = 0.0;
    double current_recovery_particle_fraction_ = 0.0;
};
