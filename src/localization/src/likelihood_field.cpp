#include "localization/likelihood_field.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <vector>

void LikelihoodField::build(
    const nav_msgs::msg::OccupancyGrid & map,
    double max_distance_m,
    double sigma_hit,
    int occupied_threshold)
{
    field_map_ = map;

    const int width = map.info.width;
    const int height = map.info.height;
    const int total_cells = width * height;
    const double resolution = map.info.resolution;
    const int max_distance_cells = static_cast<int>(std::ceil(max_distance_m / resolution));

    std::vector<int> distance_to_wall(total_cells, max_distance_cells);
    std::queue<int> cells_to_visit;

    for (int i = 0; i < total_cells; ++i) {
        if (map.data[i] > occupied_threshold) {
            distance_to_wall[i] = 0;
            cells_to_visit.push(i);
        }
    }

    const int neighbor_offsets[4][2] = {
        {1, 0},
        {-1, 0},
        {0, 1},
        {0, -1}
    };

    while (!cells_to_visit.empty()) {
        const int index = cells_to_visit.front();
        cells_to_visit.pop();

        const int row = index / width;
        const int col = index % width;
        const int next_distance = distance_to_wall[index] + 1;

        if (next_distance > max_distance_cells) {
            continue;
        }

        for (const auto & offset : neighbor_offsets) {
            const int next_col = col + offset[0];
            const int next_row = row + offset[1];

            if (next_col < 0 || next_row < 0 || next_col >= width || next_row >= height) {
                continue;
            }

            const int next_index = next_row * width + next_col;
            if (next_distance >= distance_to_wall[next_index]) {
                continue;
            }

            distance_to_wall[next_index] = next_distance;
            cells_to_visit.push(next_index);
        }
    }

    field_map_.data.assign(total_cells, 0);

    // Bake the Gaussian sensor model into the field: store the hit probability
    // exp(-d^2 / 2*sigma_hit^2) so the scorer can read it as a probability
    // directly. With sigma_hit ~ 0.2 m the value decays to ~0 well within the
    // truncation distance, so cells past the BFS frontier correctly store ~0.
    const double two_sigma_sq = 2.0 * sigma_hit * sigma_hit;

    for (int i = 0; i < total_cells; ++i) {
        const double distance_m = distance_to_wall[i] * resolution;
        const double likelihood = std::exp(-(distance_m * distance_m) / two_sigma_sq);
        field_map_.data[i] = static_cast<int8_t>(std::round(likelihood * 100.0));
    }
}

bool LikelihoodField::hasMap() const
{
    return !field_map_.data.empty();
}

bool LikelihoodField::worldToGrid(double x, double y, int & col, int & row) const
{
    if (!hasMap()) {
        return false;
    }

    const double resolution = field_map_.info.resolution;
    const double origin_x = field_map_.info.origin.position.x;
    const double origin_y = field_map_.info.origin.position.y;

    col = static_cast<int>(std::floor((x - origin_x) / resolution));
    row = static_cast<int>(std::floor((y - origin_y) / resolution));

    return col >= 0 &&
        row >= 0 &&
        col < static_cast<int>(field_map_.info.width) &&
        row < static_cast<int>(field_map_.info.height);
}

double LikelihoodField::valueAtWorld(double x, double y) const
{
    int col = 0;
    int row = 0;
    if (!worldToGrid(x, y, col, row)) {
        return 0.0;
    }

    const int index = row * field_map_.info.width + col;
    return field_map_.data[index] / 100.0;
}

const nav_msgs::msg::OccupancyGrid & LikelihoodField::message() const
{
    return field_map_;
}

