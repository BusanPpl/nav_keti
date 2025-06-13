#include "nav_keti/rrt.h"
#include <cmath>
#include <limits>
#include <random>

namespace RRT
{
    Planner::Planner(double expand_dist, double path_res, double goal_sample_rate, int max_iter)
        : expand_dist_(expand_dist), path_res_(path_res), goal_sample_rate_(goal_sample_rate), max_iter_(max_iter), gen_(rd_()) {}

    void Planner::setStart(const Vec2d& start) { start_ = start; }
    void Planner::setGoal(const Vec2d& goal) { goal_ = goal; }
    void Planner::setObstacles(const std::vector<std::tuple<double, double, double>>& obs) { obstacles_ = obs; }
    void Planner::setBounds(double xmin, double xmax, double ymin, double ymax) {
        xmin_ = xmin; xmax_ = xmax; ymin_ = ymin; ymax_ = ymax;
    }

    Vec2d Planner::getRandomPoint() {
        std::uniform_real_distribution<double> x_dist(xmin_, xmax_);
        std::uniform_real_distribution<double> y_dist(ymin_, ymax_);
        return (std::rand() % 100 < goal_sample_rate_) ? goal_ : Vec2d(x_dist(gen_), y_dist(gen_));
    }

    Node* Planner::getNearestNode(const Vec2d& pt) {
        double min_dist = std::numeric_limits<double>::max();
        Node* nearest = nullptr;
        for (auto* node : nodes_) {
            double d = std::hypot(node->position.x - pt.x, node->position.y - pt.y);
            if (d < min_dist) {
                min_dist = d;
                nearest = node;
            }
        }
        return nearest;
    }

    Node* Planner::steer(Node* from, const Vec2d& to, double extend_dist) {
        double d = std::hypot(to.x - from->position.x, to.y - from->position.y);
        double theta = atan2(to.y - from->position.y, to.x - from->position.x);

        Node* new_node = new Node(from->position);
        int steps = std::min(static_cast<int>(d / path_res_), static_cast<int>(extend_dist / path_res_));

        for (int i = 0; i < steps; ++i) {
            new_node->position.x += path_res_ * cos(theta);
            new_node->position.y += path_res_ * sin(theta);
            new_node->path.push_back(new_node->position);
        }
        new_node->parent = from;
        return new_node;
    }

    bool Planner::checkCollision(const Node* node) {
        for (const auto& obs : obstacles_) {
            double ox, oy, r;
            std::tie(ox, oy, r) = obs;
            for (const auto& p : node->path) {
                if (std::hypot(p.x - ox, p.y - oy) < r) return false;
            }
        }
        return true;
    }

    bool Planner::isGoalReached(const Node* node) {
        return std::hypot(node->position.x - goal_.x, node->position.y - goal_.y) <= expand_dist_;
    }

    std::vector<Vec2d> Planner::generateFinalPath(Node* last_node) {
        std::vector<Vec2d> path;
        while (last_node) {
            path.push_back(last_node->position);
            last_node = last_node->parent;
        }
        return path;
    }

    std::vector<Vec2d> Planner::plan() {
        
        nodes_.clear();
        nodes_.push_back(new Node(start_));

        for (int i = 0; i < max_iter_; ++i) {
            Vec2d rnd = getRandomPoint();
            Node* nearest = getNearestNode(rnd);
            Node* new_node = steer(nearest, rnd, expand_dist_);

            if (checkCollision(new_node)) {
                nodes_.push_back(new_node);
                if (isGoalReached(new_node)) return generateFinalPath(new_node);
            }
        }
        return {};
    }
    
}
