#pragma once

#include <cstdint>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>

namespace red::navigator {

struct FrontierGoal {
    // Goal position in the map frame.
    double x_map{0.0};
    double y_map{0.0};
};

class FrontierExplorer {
  public:
    FrontierExplorer();

    bool find_frontier_goal(const nav_msgs::msg::OccupancyGrid &map, double robot_x_map,
                            double robot_y_map, FrontierGoal &goal) const;
    bool is_frontier_goal(const nav_msgs::msg::OccupancyGrid &map, const FrontierGoal &goal) const;
    bool world_to_grid(const nav_msgs::msg::OccupancyGrid &map, double wx, double wy, int &gx,
                       int &gy) const;
    int get_free_threshold() const;

  private:
    bool has_unknown_neighbor(const nav_msgs::msg::OccupancyGrid &map, int x, int y) const;
    bool is_free(int8_t value) const;
    static bool is_unknown(int8_t value);

    double min_frontier_distance{0.75};
    double max_frontier_distance{30.0};
    int free_threshold{10};
    double keep_obstacle_distance{1.3};
};

} // namespace red::navigator
