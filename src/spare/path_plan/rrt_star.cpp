#include "nav_keti/rrt_star.h"

namespace RRTS
{
    Planner::Planner(int expand_dist, int goal_sample_rate, int max_iter)
        : expand_dist_(expand_dist), goal_sample_rate_(goal_sample_rate), max_iter_(max_iter),
          gen_(rd_()), search_radius_(expand_dist * 2) {}

    void Planner::setStart(const Vec2i& start) { start_ = start; }
    void Planner::setGoal(const Vec2i& goal) { goal_ = goal; }

    void Planner::setCostmapData(
        const std::vector<uint8_t>& data,
        int size_x, int size_y,
        double resolution,
        double origin_x, double origin_y,
        double robot_x, double robot_y)
    {
        costmap_data_ = data;
        map_size_x_ = size_x;
        map_size_y_ = size_y;
    
        obstacles_.clear();
        for (size_t i = 0; i < data.size(); ++i) {
            if (data[i] > 100) {
                int x = i % size_x;
                int y = i / size_x;
                obstacles_.emplace_back(x, y);
            }
        }
    }

    Vec2i Planner::getRandomPoint() {
        std::uniform_int_distribution<int> x_dist(0, map_size_x_ - 1);
        std::uniform_int_distribution<int> y_dist(0, map_size_y_ - 1);
        return (std::rand() % 100 < goal_sample_rate_) ? goal_ : Vec2i(x_dist(gen_), y_dist(gen_));
    }

    Node* Planner::getNearestNode(const Vec2i& pt) {
        double min_dist = std::numeric_limits<double>::max();
        Node* nearest = nullptr;
        for (auto* node : nodes_) {
            double d = calcDistance(node->position, pt);
            if (d < min_dist) {
                min_dist = d;
                nearest = node;
            }
        }
        return nearest;
    }

    Node* Planner::steer(Node* from, const Vec2i& to, int extend_dist) {
        int dx = to.x - from->position.x;
        int dy = to.y - from->position.y;
        double d = std::hypot(dx, dy);
        if (d == 0) return nullptr;

        int steps = std::min(static_cast<int>(d), extend_dist);
        if (steps <= 0) return nullptr;

        Node* new_node = new Node(from->position);
        for (int i = 0; i < steps; ++i) {
            double t = static_cast<double>(i) / d;
            int x = static_cast<int>(from->position.x + t * dx);
            int y = static_cast<int>(from->position.y + t * dy);
            new_node->path.emplace_back(x, y);
        }
        new_node->position = Vec2i(from->position.x + dx * steps / d,
                                   from->position.y + dy * steps / d);
        new_node->parent = from;
        new_node->cost = from->cost + d;
        return new_node;
    }

    bool Planner::checkCollision(const Node* node) {
        for (const auto& p : node->path) {
            if (p.x < 0 || p.x >= map_size_x_ || p.y < 0 || p.y >= map_size_y_)
                return false;

            int idx = p.y * map_size_x_ + p.x;
            if (costmap_data_[idx] > 100)
                return false;
        }
        return true;
    }

    bool Planner::isGoalReached(const Node* node) {
        return calcDistance(node->position, goal_) <= expand_dist_;
    }

    double Planner::calcDistance(const Vec2i& a, const Vec2i& b) {
        return std::hypot(a.x - b.x, a.y - b.y);
    }

    std::vector<Node*> Planner::findNearNodes(Node* new_node) {
        std::vector<Node*> near_nodes;
        for (auto* node : nodes_) {
            if (calcDistance(node->position, new_node->position) <= search_radius_) {
                near_nodes.push_back(node);
            }
        }
        return near_nodes;
    }

    Node* Planner::chooseBestParent(Node* new_node, const std::vector<Node*>& near_nodes) {
        Node* best_parent = new_node->parent;
        double min_cost = best_parent->cost + calcDistance(best_parent->position, new_node->position);

        for (auto* node : near_nodes) {
            Node* temp = steer(node, new_node->position, expand_dist_);
            if (!temp || !checkCollision(temp)) {
                delete temp;
                continue;
            }

            double cost = node->cost + calcDistance(node->position, new_node->position);
            if (cost < min_cost) {
                min_cost = cost;
                best_parent = node;
            }
            delete temp;
        }

        new_node->parent = best_parent;
        new_node->cost = min_cost;
        return new_node;
    }
    void Planner::rewire(Node* new_node, const std::vector<Node*>& near_nodes) {
        for (auto* node : near_nodes) {
            Node* temp = steer(new_node, node->position, expand_dist_);
            if (!temp || !checkCollision(temp)) {
                delete temp;
                continue;
            }
    
            double dist = calcDistance(new_node->position, node->position);
            double path_cost = computePathCost(temp->path);
    
            // 거리와 셀 코스트를 비율 조정해서 합산
            double new_cost = new_node->cost + dist + 0.7 * path_cost;
    
            if (new_cost < node->cost) {
                node->parent = new_node;
                node->cost = new_cost;
            }
    
            delete temp;
        }
    }
    
    double Planner::computePathCost(const std::vector<Vec2i>& path) {
        double total_cost = 0.0;
        for (const auto& p : path) {
            if (p.x < 0 || p.x >= map_size_x_ || p.y < 0 || p.y >= map_size_y_)
                return std::numeric_limits<double>::infinity();
    
            int idx = p.y * map_size_x_ + p.x;
            int cell_cost = costmap_data_[idx];
    
            // 0 ~ 100 사이 코스트를 [1.0 ~ 2.0] 정도의 가중치로 환산
            double weight = cell_cost;
            total_cost += weight;
        }
        return total_cost;
    }

    std::vector<Vec2i> Planner::generateFinalPath(Node* last_node) {
        std::vector<Vec2i> path;
        while (last_node) {
            path.push_back(last_node->position);
            last_node = last_node->parent;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
    std::vector<Vec2i> Planner::smoothPath(const std::vector<Vec2i>& path, int max_iter) {
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
    
    

    bool Planner::isLineCollisionFree(const Vec2i& p1, const Vec2i& p2) {
        int x0 = p1.x, y0 = p1.y;
        int x1 = p2.x, y1 = p2.y;
    
        int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;
    
        while (true) {
            if (x0 < 0 || x0 >= map_size_x_ || y0 < 0 || y0 >= map_size_y_)
                return false;
    
            int idx = y0 * map_size_x_ + x0;
            if (costmap_data_[idx] > 100)
                return false;
    
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 < dx)  { err += dx; y0 += sy; }
        }
    
        return true;
    }
    
    std::vector<Vec2i> Planner::plan() {
        nodes_.clear();
        Node* start_node = new Node(start_);
        start_node->cost = 0.0;
        nodes_.push_back(start_node);

        for (int i = 0; i < max_iter_; ++i) {
            Vec2i rnd = getRandomPoint();
            Node* nearest = getNearestNode(rnd);
            Node* new_node = steer(nearest, rnd, expand_dist_);
            if (!new_node || !checkCollision(new_node)) {
                delete new_node;
                continue;
            }

            auto near_nodes = findNearNodes(new_node);
            new_node = chooseBestParent(new_node, near_nodes);
            nodes_.push_back(new_node);
            rewire(new_node, near_nodes);

            if (isGoalReached(new_node)) {
                return generateFinalPath(new_node);
            }
        }

        return {};
    }

} // namespace RRTS
