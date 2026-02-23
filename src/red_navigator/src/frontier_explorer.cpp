#include "red_navigator/frontier_explorer.hpp"

#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace red::navigator {

FrontierExplorer::FrontierExplorer() = default;

bool FrontierExplorer::find_frontier_goal(const nav_msgs::msg::OccupancyGrid &map,
                                          double robot_x_map, double robot_y_map,
                                          FrontierGoal &goal) const {
    const int width = static_cast<int>(map.info.width);
    const int height = static_cast<int>(map.info.height);
    const double resolution = map.info.resolution;
    const double origin_x = map.info.origin.position.x;
    const double origin_y = map.info.origin.position.y;

    if (width <= 0 || height <= 0 || resolution <= 0.0) {
        return false;
    }
    const size_t expected = static_cast<size_t>(width) * static_cast<size_t>(height);
    if (map.data.size() < expected) {
        return false;
    }

    auto is_obstacle = [this](int8_t value) { return value > this->free_threshold; };

    const bool enforce_clearance = this->keep_obstacle_distance > 0.0;
    std::vector<double> obstacle_distance_cells;
    bool has_obstacle = false;
    if (enforce_clearance) {
        obstacle_distance_cells.assign(expected, std::numeric_limits<double>::infinity());
        struct QueueNode {
            double distance;
            int idx;
        };
        struct QueueNodeCompare {
            bool operator()(const QueueNode &lhs, const QueueNode &rhs) const {
                return lhs.distance > rhs.distance;
            }
        };
        std::priority_queue<QueueNode, std::vector<QueueNode>, QueueNodeCompare> queue;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                const int idx = y * width + x;
                const int8_t value = map.data[static_cast<size_t>(idx)];
                if (is_obstacle(value)) {
                    obstacle_distance_cells[static_cast<size_t>(idx)] = 0.0;
                    queue.push(QueueNode{0.0, idx});
                    has_obstacle = true;
                }
            }
        }

        const int clearance_offsets[8][2] = {
            {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1},
        };
        const double clearance_costs[8] = {
            1.0, 1.0, 1.0, 1.0, std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0),
        };

        while (!queue.empty()) {
            const QueueNode current = queue.top();
            queue.pop();
            const int idx = current.idx;
            if (current.distance > obstacle_distance_cells[static_cast<size_t>(idx)]) {
                continue;
            }
            const int x = idx % width;
            const int y = idx / width;
            for (size_t i = 0; i < 8; ++i) {
                const int nx = x + clearance_offsets[i][0];
                const int ny = y + clearance_offsets[i][1];
                if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
                    continue;
                }
                const int nidx = ny * width + nx;
                const double next_dist = current.distance + clearance_costs[i];
                if (next_dist >= obstacle_distance_cells[static_cast<size_t>(nidx)]) {
                    continue;
                }
                obstacle_distance_cells[static_cast<size_t>(nidx)] = next_dist;
                queue.push(QueueNode{next_dist, nidx});
            }
        }
    }

    double best_score = std::numeric_limits<double>::max();
    bool found = false;

    for (int y = 1; y < height - 1; ++y) {
        for (int x = 1; x < width - 1; ++x) {
            const int idx = y * width + x;
            const int8_t value = map.data[static_cast<size_t>(idx)];
            if (!this->is_free(value)) {
                continue;
            }
            if (!this->has_unknown_neighbor(map, x, y)) {
                continue;
            }

            const double wx = origin_x + (static_cast<double>(x) + 0.5) * resolution;
            const double wy = origin_y + (static_cast<double>(y) + 0.5) * resolution;
            const double dx = wx - robot_x_map;
            const double dy = wy - robot_y_map;
            const double dist = std::hypot(dx, dy);

            if (dist < this->min_frontier_distance) {
                continue;
            }
            if (this->max_frontier_distance > 0.0 && dist > this->max_frontier_distance) {
                continue;
            }
            if (enforce_clearance && has_obstacle) {
                const double cells = obstacle_distance_cells[static_cast<size_t>(idx)];
                if (!std::isfinite(cells)) {
                    continue;
                }
                const double obstacle_distance = cells * resolution;
                if (obstacle_distance <= this->keep_obstacle_distance) {
                    continue;
                }
            }

            const double score = dist;
            if (score < best_score) {
                best_score = score;
                goal.x_map = wx;
                goal.y_map = wy;
                found = true;
            }
        }
    }

    return found;
}

bool FrontierExplorer::is_frontier_goal(const nav_msgs::msg::OccupancyGrid &map,
                                        const FrontierGoal &goal) const {
    int gx = 0;
    int gy = 0;
    if (!this->world_to_grid(map, goal.x_map, goal.y_map, gx, gy)) {
        return false;
    }
    const int width = static_cast<int>(map.info.width);
    const int height = static_cast<int>(map.info.height);
    if (width <= 0 || height <= 0) {
        return false;
    }
    const int idx = gy * width + gx;
    if (idx < 0 || static_cast<size_t>(idx) >= map.data.size()) {
        return false;
    }
    if (!this->is_free(map.data[static_cast<size_t>(idx)])) {
        return false;
    }
    return this->has_unknown_neighbor(map, gx, gy);
}

bool FrontierExplorer::world_to_grid(const nav_msgs::msg::OccupancyGrid &map, double wx, double wy,
                                     int &gx, int &gy) const {
    const double resolution = map.info.resolution;
    if (resolution <= 0.0) {
        return false;
    }
    const double origin_x = map.info.origin.position.x;
    const double origin_y = map.info.origin.position.y;
    const int width = static_cast<int>(map.info.width);
    const int height = static_cast<int>(map.info.height);

    const double dx = wx - origin_x;
    const double dy = wy - origin_y;
    gx = static_cast<int>(std::floor(dx / resolution));
    gy = static_cast<int>(std::floor(dy / resolution));

    return gx >= 0 && gx < width && gy >= 0 && gy < height;
}

bool FrontierExplorer::has_unknown_neighbor(const nav_msgs::msg::OccupancyGrid &map, int x,
                                            int y) const {
    const int width = static_cast<int>(map.info.width);
    const int height = static_cast<int>(map.info.height);
    if (width <= 0 || height <= 0) {
        return false;
    }
    const size_t expected = static_cast<size_t>(width) * static_cast<size_t>(height);
    if (map.data.size() < expected) {
        return false;
    }
    auto index = [width](int ix, int iy) { return iy * width + ix; };

    const int neighbor_offsets[4][2] = {
        {1, 0},
        {-1, 0},
        {0, 1},
        {0, -1},
    };

    for (const auto &offset : neighbor_offsets) {
        const int nx = x + offset[0];
        const int ny = y + offset[1];
        if (nx <= 0 || ny <= 0 || nx >= width - 1 || ny >= height - 1) {
            continue;
        }
        const int nidx = index(nx, ny);
        if (nidx < 0 || static_cast<size_t>(nidx) >= map.data.size()) {
            continue;
        }
        if (this->is_unknown(map.data[static_cast<size_t>(nidx)])) {
            return true;
        }
    }
    return false;
}

bool FrontierExplorer::is_free(int8_t value) const {
    return value >= 0 && value <= this->free_threshold;
}

bool FrontierExplorer::is_unknown(int8_t value) { return value < 0; }

int FrontierExplorer::get_free_threshold() const { return this->free_threshold; }

} // namespace red::navigator
