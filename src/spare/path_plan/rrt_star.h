#pragma once

#include <vector>
#include <tuple>
#include <random>
#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream>

namespace RRTS
{
    struct Vec2i {
        int x, y;
        Vec2i(int x_ = 0, int y_ = 0) : x(x_), y(y_) {}
        bool operator==(const Vec2i& other) const {
            return x == other.x && y == other.y;
        }
    };

    struct Node {
        Vec2i position;
        std::vector<Vec2i> path;
        Node* parent = nullptr;
        double cost = 0.0;

        Node(const Vec2i& pos) : position(pos) {}
    };

    class Planner {
    public:
        Planner(int expand_dist, int goal_sample_rate, int max_iter);

        void setStart(const Vec2i& start);
        void setGoal(const Vec2i& goal);
        void setCostmapData(
            const std::vector<uint8_t>& data,
            int size_x, int size_y,
            double resolution,
            double origin_x, double origin_y,
            double robot_x, double robot_y);
        std::vector<Vec2i> plan();
        std::vector<Vec2i> smoothPath(const std::vector<Vec2i>& path, int max_iter);

    private:
        double computePathCost(const std::vector<Vec2i>& path);
        Vec2i getRandomPoint();
        Node* getNearestNode(const Vec2i& pt);
        Node* steer(Node* from, const Vec2i& to, int extend_dist);
        bool checkCollision(const Node* node);
        bool isGoalReached(const Node* node);
        std::vector<Node*> findNearNodes(Node* node);
        Node* chooseBestParent(Node* new_node, const std::vector<Node*>& near_nodes);
        void rewire(Node* new_node, const std::vector<Node*>& near_nodes);
        double calcDistance(const Vec2i& a, const Vec2i& b);
        std::vector<Vec2i> generateFinalPath(Node* last_node);
        
        bool isLineCollisionFree(const Vec2i& p1, const Vec2i& p2);

        Vec2i start_, goal_;
        std::vector<Node*> nodes_;
        std::vector<Vec2i> obstacles_;
        std::vector<uint8_t> costmap_data_;

        int expand_dist_, goal_sample_rate_, max_iter_;
        int map_size_x_, map_size_y_;
        double search_radius_;

        std::random_device rd_;
        std::mt19937 gen_;
    };
}
