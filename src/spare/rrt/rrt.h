#ifndef RRT_HPP
#define RRT_HPP

#include <vector>
#include <cmath>
#include <tuple>
#include <limits>
#include <random>

namespace RRT
{
    struct Vec2d {
        double x, y;
        Vec2d(double _x = 0, double _y = 0) : x(_x), y(_y) {}
    };

    struct Node {
        Vec2d position;
        std::vector<Vec2d> path;
        Node* parent;

        Node(Vec2d pos, Node* p = nullptr) : position(pos), parent(p) {}
    };

    class Planner {
    public:
        Planner(double expand_dist, double path_res, double goal_sample_rate, int max_iter);
        void setStart(const Vec2d& start);
        void setGoal(const Vec2d& goal);
        void setObstacles(const std::vector<std::tuple<double, double, double>>& obs);
        void setBounds(double xmin, double xmax, double ymin, double ymax);
        std::vector<Vec2d> plan();

    private:
        Vec2d getRandomPoint();
        Node* getNearestNode(const Vec2d& pt);
        Node* steer(Node* from, const Vec2d& to, double extend_dist);
        bool checkCollision(const Node* node);
        bool isGoalReached(const Node* node);
        std::vector<Vec2d> generateFinalPath(Node* last_node);

        std::vector<Node*> nodes_;
        std::vector<std::tuple<double, double, double>> obstacles_;
        Vec2d start_, goal_;
        double xmin_, xmax_, ymin_, ymax_;
        double expand_dist_, path_res_;
        double goal_sample_rate_;
        int max_iter_;
        std::random_device rd_;
        std::mt19937 gen_;
    };
}

#endif
