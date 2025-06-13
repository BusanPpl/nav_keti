#ifndef RRT_STAR_PLANNER_H
#define RRT_STAR_PLANNER_H

#include "path_planner.h"
#include <random>

class RRTStarPlanner : public PathPlannerInterface {
public:
    RRTStarPlanner(int expand_dist, int max_iter, int goal_sample_rate);
    ~RRTStarPlanner() = default;
    std::vector<Vec2i> plan() override;
    std::vector<Vec2i> smoothPath(const std::vector<Vec2i>& path, int max_iter) override;
    
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
    double resolution_;
    int expand_dist_, max_iter_, goal_sample_rate_;
    std::random_device rd_;
    std::mt19937 gen_{rd_()};

    // 내부 기능 함수
    bool inBounds(const Vec2i& p) const;
    bool isObstacle(const Vec2i& p) const;
    double dist(const Vec2i& a, const Vec2i& b) const;
    Vec2i getRandom();
    Node* getNearest(const std::vector<Node*>& nodes, const Vec2i& p);
    Node* steer(Node* from, const Vec2i& to);
    bool checkCollision(Node* n) const;
    std::vector<Node*> findNear(const std::vector<Node*>& nodes, Node* new_node);
    Node* chooseBestParent(Node* new_node, const std::vector<Node*>& near);
    void rewire(Node* new_node, const std::vector<Node*>& near);
    bool isLineCollisionFree(const Vec2i& p1, const Vec2i& p2);
};

#endif
