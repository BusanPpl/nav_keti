#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H

#include "path_planner.h"
#include <functional>

class AStarPlanner : public PathPlannerInterface {
public:
    using HeuristicFunc = std::function<unsigned int(const Vec2i&, const Vec2i&)>;
    std::vector<Vec2i> plan() override;
    void setHeuristic(HeuristicFunc h_func);
    void setStart(const Vec2i& start) override { start_ = start; }
    void setGoal(const Vec2i& goal) override { goal_ = goal; }
    void setCostmapData(const std::vector<uint8_t>& data, int size_x, int size_y, double resolution) override {
        costmap_ = data;
        size_x_ = size_x;
        size_y_ = size_y;
        resolution_ = resolution;
    }
private:
    Vec2i start_, goal_;
    std::vector<uint8_t> costmap_;
    int size_x_, size_y_;
    HeuristicFunc heuristic_ = euclidean;

    static unsigned int manhattan(const Vec2i& a, const Vec2i& b);
    static unsigned int euclidean(const Vec2i& a, const Vec2i& b);
    static unsigned int octagonal(const Vec2i& a, const Vec2i& b);

    bool inBounds(const Vec2i& p) const;
    bool isObstacle(const Vec2i& p) const;
    unsigned int calculateCost(const Vec2i& current, const Vec2i& next) const;
};

#endif
