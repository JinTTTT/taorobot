#pragma once

#include "nav_msgs/msg/occupancy_grid.hpp"

class LikelihoodField
{
public:
    // Build the likelihood field. Each cell stores the beam-endpoint hit
    // probability exp(-d^2 / 2*sigma_hit^2), where d is the metric distance from
    // the cell to the nearest occupied cell (the Gaussian sensor model is baked
    // in at build time so the scorer can use the looked-up value directly). The
    // distance transform is truncated at max_distance_m; cells beyond it store
    // ~0. Stored quantized to 0..100.
    void build(
        const nav_msgs::msg::OccupancyGrid & map,
        double max_distance_m,
        double sigma_hit,
        int occupied_threshold = 50);

    bool hasMap() const;
    bool worldToGrid(double x, double y, int & col, int & row) const;
    double valueAtWorld(double x, double y) const;

    const nav_msgs::msg::OccupancyGrid & message() const;

private:
    nav_msgs::msg::OccupancyGrid field_map_;
};

