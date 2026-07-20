/// OccupancyMapper owns a log-odds occupancy grid and integrates laser scans into it.
///
/// Public API:
/// - configure()/clear(): initialize or reset the grid map.
/// - updateWithScan()/updateWithScanData(): integrate laser measurements into map_log_odds_.
/// - takeAndClearMapChanges(): return cells whose occupied/free state changed.
/// - buildOccupancyGridMsg(): export the internal map as a ROS OccupancyGrid.
///
/// Scan integration flow:
///   updateWithScan()
///     -> updateWithScanData()
///       -> bresenhamLine()
///       -> updateCellLogOddsAndTrackChange()
///         -> update map_log_odds_
///         -> record changed cells
///
/// FastSLAM likelihood-field update:
///   takeAndClearMapChanges()
///     -> pass changed cells to FastSLAM
///     -> clear temporary change buffers
///   likelihood_field.updateFromMapChanges()
///     -> update the likelihood field from those changed cells
#pragma once

#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <string>

#include <cstdint>
#include <utility>
#include <vector>

class OccupancyMapper
{
public:
  struct ScanData
  {
    std::vector<float> ranges;
    float angle_min = 0.0F;
    float angle_increment = 0.0F;
    float range_min = 0.0F;
    float range_max = 0.0F;
  };

  struct Config
  {
    double resolution = 0.05;
    int width = 500;
    int height = 500;
    double origin_x = 0.0;
    double origin_y = 0.0;
    double hit_probability = 0.90;
    double free_probability = 0.05;
    double log_odds_min = -10.0;
    double log_odds_max = 10.0;
    bool publish_unknown_for_unobserved = true;   // never-observed cells -> -1 (standard "unknown")
  };

  // Cells whose occupancy state crossed zero during scan integration.
  // FastSLAM uses these lists to update the likelihood field incrementally.
  struct MapChanges
  {
    std::vector<int> cells_changed_to_occupied;
    std::vector<int> cells_changed_to_free;
  };

  void configure(const Config & config);
  void clear();

  bool worldToGrid(double wx, double wy, int & gx, int & gy) const;
  bool isValidCell(int x, int y) const;
  void updateWithScan(
    const sensor_msgs::msg::LaserScan & scan,
    double robot_x,
    double robot_y,
    double robot_theta);
  void updateWithScanData(
    const ScanData & scan,
    double robot_x,
    double robot_y,
    double robot_theta);
  nav_msgs::msg::OccupancyGrid buildOccupancyGridMsg(
    const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp) const;

  // Move out the cells changed by recent scan updates, then empty the internal buffers.
  MapChanges takeAndClearMapChanges();

  const Config & getConfig() const;
  const std::vector<double> & getLogOdds() const;

private:
  std::vector<std::pair<int, int>> bresenhamLine(int x0, int y0, int x1, int y1) const;
  int gridToIndex(int x, int y) const;
  double clamp(double value, double low, double high) const;
  void updateCellLogOddsAndTrackChange(int index, double log_odds_delta);

  Config config_;
  // initialization, later will be replaced by configuration
  double log_odds_hit_ = 0.0;
  double log_odds_pass_ = 0.0;

  // the real map: One log-odds value per grid cell
  std::vector<double> map_log_odds_;

  // Temporary buffers filled while integrating scans and drained by takeAndClearMapChanges().
  std::vector<int> cells_changed_to_occupied_;
  std::vector<int> cells_changed_to_free_;
};
