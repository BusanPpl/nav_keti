#include "nav_keti/rrt_star_planner.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>

RRTStarPlanner::RRTStarPlanner(int expand_dist, int max_iter, int goal_sample_rate)
    : expand_dist_(expand_dist),
      max_iter_(max_iter),
      goal_sample_rate_(goal_sample_rate) {}

std::vector<Vec2i> RRTStarPlanner::plan() {

    std::vector<Node*> nodes;
    Node* start_node = new Node(start_);
    nodes.push_back(start_node);
    
    for (int i = 0; i < max_iter_; ++i) {
        Vec2i rnd = getRandom();

        Node* nearest = getNearest(nodes, rnd);
        if (!nearest) {
            // std::cout << "nearest node not found" << std::endl;
            continue;
        }

        Node* new_node = steer(nearest, rnd);
        if (!new_node) {
            // std::cout << "[RRT] Steer failed" << std::endl;
            continue;
        }

        if (!checkCollision(new_node)) {
            delete new_node;
            continue;
        }

        auto near_nodes = findNear(nodes, new_node);
        new_node = chooseBestParent(new_node, near_nodes);
        nodes.push_back(new_node);
        rewire(new_node, near_nodes);

        if (dist(new_node->pos, goal_) <= expand_dist_) {
            std::vector<Vec2i> path;
            for (Node* cur = new_node; cur; cur = cur->parent)
                path.push_back(cur->pos);
            std::reverse(path.begin(), path.end());
            for (auto* n : nodes) delete n;
            return path;
        }
    }
    for (auto* n : nodes) delete n;
    return {};
}

bool RRTStarPlanner::inBounds(const Vec2i& p) const {
    return p.x >= 0 && p.x < size_x_ && p.y >= 0 && p.y < size_y_;
}


bool RRTStarPlanner::isObstacle(const Vec2i& p) const {
    if (!inBounds(p)) return true;
    int index = p.y * size_x_ + p.x;
    if (index < 0 || index >= static_cast<int>(costmap_.size())) {
        std::cout << "[isObstacle] Out of bounds: (" << p.x << ", " << p.y << "), index=" << index << std::endl;
        return true;
    }
    return costmap_[index] > 100 && costmap_[index] != 255;
}

double RRTStarPlanner::dist(const Vec2i& a, const Vec2i& b) const {
    return std::hypot(a.x - b.x, a.y - b.y);
}

Vec2i RRTStarPlanner::getRandom() {
    if (size_x_ <= 0 || size_y_ <= 0) {
        return goal_;
    }
    std::uniform_int_distribution<int> x(0, size_x_ - 1);
    std::uniform_int_distribution<int> y(0, size_y_ - 1);
    std::uniform_int_distribution<int> prob(0, 99);
    return (prob(gen_) < goal_sample_rate_) ? goal_ : Vec2i(x(gen_), y(gen_));
}

Node* RRTStarPlanner::getNearest(const std::vector<Node*>& nodes, const Vec2i& point) {
    if (nodes.empty()) {
        return nullptr;
    }

    Node* nearest = nullptr;
    double min_dist = std::numeric_limits<double>::max();

    for (auto* node : nodes) {
        double d = dist(node->pos, point);
        if (d < min_dist) {
            min_dist = d;
            nearest = node;
        }
    }

    return nearest;
}

Node* RRTStarPlanner::steer(Node* from, const Vec2i& to) {
    int dx = to.x - from->pos.x, dy = to.y - from->pos.y;
    double d = std::hypot(dx, dy);
    if (d == 0) return nullptr;
    int steps = std::min(static_cast<int>(d), expand_dist_);
    Node* new_node = new Node(from->pos);
    for (int i = 0; i < steps; ++i) {
        double t = static_cast<double>(i) / d;
        int x = static_cast<int>(from->pos.x + t * dx);
        int y = static_cast<int>(from->pos.y + t * dy);
        new_node->path.push_back({x, y});
    }
    new_node->pos = {from->pos.x + dx * steps / d, from->pos.y + dy * steps / d};
    new_node->parent = from;
    new_node->cost = from->cost + d;
    return new_node;
}

bool RRTStarPlanner::checkCollision(Node* n) const {
    for (const auto& p : n->path) {
        int idx = p.y * size_x_ + p.x;
        // std::cout << "[DEBUG] Check obstacle at (" << p.x << ", " << p.y << ") = "
        //           << (inBounds(p) ? std::to_string(costmap_[idx]) : "OUT OF BOUNDS") << std::endl;

        if (!inBounds(p) || isObstacle(p)) return false;
    }
    return true;
}

std::vector<Node*> RRTStarPlanner::findNear(const std::vector<Node*>& nodes, Node* new_node) {
    std::vector<Node*> near;
    for (auto* n : nodes)
        if (dist(n->pos, new_node->pos) <= expand_dist_ * 2)
            near.push_back(n);
    return near;
}

Node* RRTStarPlanner::chooseBestParent(Node* new_node, const std::vector<Node*>& near) {
    Node* best = new_node->parent;
    double min_cost = best ? best->cost + dist(best->pos, new_node->pos) : std::numeric_limits<double>::max();
    for (auto* n : near) {
        Node* tmp = steer(n, new_node->pos);
        if (!tmp || !checkCollision(tmp)) {
            delete tmp;
            continue;
        }
        double cost = n->cost + dist(n->pos, new_node->pos);
        if (cost < min_cost) {
            min_cost = cost;
            best = n;
        }
        delete tmp;
    }
    new_node->parent = best;
    new_node->cost = min_cost;
    return new_node;
}

void RRTStarPlanner::rewire(Node* new_node, const std::vector<Node*>& near) {
    for (auto* n : near) {
        Node* tmp = steer(new_node, n->pos);
        if (!tmp || !checkCollision(tmp)) {
            delete tmp;
            continue;
        }
        double cost = new_node->cost + dist(new_node->pos, n->pos);
        if (cost < n->cost) {
            n->parent = new_node;
            n->cost = cost;
        }
        delete tmp;
    }
}

std::vector<Vec2i> RRTStarPlanner::smoothPath(const std::vector<Vec2i>& path, int max_iter) {
    if (path.size() < 3) return path;

    std::vector<Vec2i> new_path = path;

    for (int i = 0; i < max_iter; ++i) {
        int len = new_path.size();
        if (len < 3) break;

        int max_idx1 = len - 3;
        if (max_idx1 <= 0) break;

        int idx1 = std::rand() % max_idx1;
        int remaining = len - idx1 - 2;
        if (remaining <= 0) continue;

        int idx2 = idx1 + 2 + std::rand() % remaining;
        if (idx2 >= len) continue;

        const Vec2i& p1 = new_path[idx1];
        const Vec2i& p2 = new_path[idx2];

        if (!isLineCollisionFree(p1, p2)) continue;

        std::vector<Vec2i> temp_path;
        temp_path.insert(temp_path.end(), new_path.begin(), new_path.begin() + idx1 + 1);
        temp_path.push_back(p2);
        temp_path.insert(temp_path.end(), new_path.begin() + idx2 + 1, new_path.end());

        new_path = temp_path;
    }

    // 마지막 지점 보장: goal이 없으면 추가
    if (!new_path.empty() && !(new_path.back() == path.back())) {
        new_path.push_back(path.back());
    }

    return new_path;
}




bool RRTStarPlanner::isLineCollisionFree(const Vec2i& p1, const Vec2i& p2) {
    int x0 = p1.x, y0 = p1.y;
    int x1 = p2.x, y1 = p2.y;

    int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        if (x0 < 0 || x0 >= size_x_ || y0 < 0 || y0 >= size_y_)
            return false;

        int idx = y0 * size_x_ + x0;
        if (costmap_[idx] > 50 && costmap_[idx] != 255)
            return false;

        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx)  { err += dx; y0 += sy; }
    }

    return true;
}