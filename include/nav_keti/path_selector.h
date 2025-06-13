#ifndef PATH_SELECTOR_H
#define PATH_SELECTOR_H

#include <vector>
#include <cstdint>
#include <string>
#include <memory>
#include "path_planner.h"
#include "astar_planner.h"
#include "rrt_star_planner.h"
#include <iostream>

class Planner {
    
public:
    Planner(const std::string& mode) {
        setMode(mode);
    }
    void setMode(const std::string& mode) {
        mode_ = mode;
        if (mode_ == "astar") {
            planner_core_ = std::make_unique<AStarPlanner>();
        } else if (mode_ == "rrt_star") {
            planner_core_ = std::make_unique<RRTStarPlanner>(10,50000,30);
        }
    }

    void setStart(const Vec2i& start) {
        start_ = start;

        if (planner_core_) planner_core_->setStart(start_);
    }

    void setGoal(const Vec2i& goal) {
        goal_ = goal;
        if (planner_core_) planner_core_->setGoal(goal_);
    }

    void setCostmapData(const std::vector<uint8_t>& data, int size_x, int size_y, double resolution) {
        costmap_ = data;
        size_x_ = size_x;
        size_y_ = size_y;
        resolution_ = resolution;
        if (planner_core_) planner_core_->setCostmapData(costmap_, size_x_, size_y_, resolution_);
    }

    std::vector<Vec2i> plan() {
        if (planner_core_) return planner_core_->plan();
        return {};
    }
    std::vector<Vec2i> smoothPath(const std::vector<Vec2i>& path, int max_iter) {
        if (planner_core_) return planner_core_->smoothPath(path, max_iter);
        return path;
    }
private:

    Vec2i start_, goal_;
    std::vector<uint8_t> costmap_;
    int size_x_ = 0, size_y_ = 0;
    double resolution_ = 0.0;
    std::string mode_;
    std::unique_ptr<PathPlannerInterface> planner_core_;
};

#endif // PATH_SELECTOR_H
