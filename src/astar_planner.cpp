#include "nav_keti/astar_planner.h"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <iostream>
// void AStarPlanner::setHeuristic(HeuristicFunc h_func) {
//     heuristic_ = h_func;
// }

// std::vector<Vec2i> AStarPlanner::plan() {
//     std::vector<Node*> open, closed;
//     Node* start_node = new Node(start_);
//     open.push_back(start_node);

//     std::vector<Vec2i> directions = {{1,0},{-1,0},{0,1},{0,-1},{-1,-1},{1,1},{-1,1},{1,-1}};

//     auto findNode = [&](std::vector<Node*>& list, const Vec2i& p) {
//         for (auto* n : list) if (n->pos == p) return n;
//         return (Node*)nullptr;
//     };

//     while (!open.empty()) {
//         std::sort(open.begin(), open.end(), [](Node* a, Node* b) { return a->f() < b->f(); });
//         Node* current = open.front();

//         if (current->pos == goal_) {
//             std::vector<Vec2i> path;
//             while (current) {
//                 path.push_back(current->pos);
//                 current = current->parent;
//             }
//             std::reverse(path.begin(), path.end());
//             for (auto* n : open) delete n;
//             for (auto* n : closed) delete n;
//             return path;
//         }

//         open.erase(open.begin());
//         closed.push_back(current);

//         for (const auto& dir : directions) {
//             Vec2i next = { current->pos.x + dir.x, current->pos.y + dir.y };
//             if (!inBounds(next) || isObstacle(next) || findNode(closed, next)) continue;

//             Node* successor = new Node(next, current);
//             successor->g = current->g + calculateCost(current->pos, next);
//             successor->h = heuristic_(next, goal_);

//             Node* existing = findNode(open, next);
//             if (!existing) open.push_back(successor);
//             else if (successor->f() < existing->f()) {
//                 existing->g = successor->g;
//                 existing->parent = current;
//                 delete successor;
//             } else delete successor;
//         }
//     }

//     for (auto* n : open) delete n;
//     for (auto* n : closed) delete n;
//     return {};
// }
void AStarPlanner::setHeuristic(HeuristicFunc h_func) {
    heuristic_ = h_func;
}

std::vector<Vec2i> AStarPlanner::plan() {
    using Clock = std::chrono::steady_clock;
    auto start_time = Clock::now(); // 시작 시간 측정
    //std::cout<<"start_time: "<<std::chrono::duration_cast<std::chrono::milliseconds>(start_time.time_since_epoch()).count()<<std::endl; 
    std::vector<Node*> open, closed;
    Node* start_node = new Node(start_);
    open.push_back(start_node);
    int step = 1;
    std::vector<Vec2i> directions = {{step,0},{-step,0},{0,step},{0,-step},{-step,-step},{step,step},{-step,step},{step,-step}};

    auto findNode = [&](std::vector<Node*>& list, const Vec2i& p) {
        for (auto* n : list) if (n->pos == p) return n;
        return (Node*)nullptr;
    };

    while (!open.empty()) {
        auto now = Clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
        if (elapsed > 1000) {
            // 현재 best 노드에서 경로 추적
            Node* best = closed.empty() ? open.front() : closed.back();
            std::vector<Vec2i> partial_path;
            while (best) {
                partial_path.push_back(best->pos);
                best = best->parent;
            }
            std::reverse(partial_path.begin(), partial_path.end());
            for (auto* n : open) delete n;
            for (auto* n : closed) delete n;
            return partial_path;
            std::cout<<"elapsed: "<<elapsed<<std::endl;
        }

        std::sort(open.begin(), open.end(), [](Node* a, Node* b) { return a->f() < b->f(); });
        Node* current = open.front();

        if (current->pos == goal_) {
            std::vector<Vec2i> path;
            while (current) {
                path.push_back(current->pos);
                current = current->parent;
            }
            std::reverse(path.begin(), path.end());
            for (auto* n : open) delete n;
            for (auto* n : closed) delete n;
            return path;
        }

        open.erase(open.begin());
        closed.push_back(current);

        for (const auto& dir : directions) {
            Vec2i next = { current->pos.x + dir.x, current->pos.y + dir.y };
            if (!inBounds(next) || isObstacle(next) || findNode(closed, next)) continue;

            Node* successor = new Node(next, current);
            successor->g = current->g + calculateCost(current->pos, next);
            successor->h = heuristic_(next, goal_) * 15;

            Node* existing = findNode(open, next);
            if (!existing) open.push_back(successor);
            else if (successor->f() < existing->f()) {
                existing->g = successor->g;
                existing->parent = current;
                delete successor;
            } else delete successor;
        }
    }

    for (auto* n : open) delete n;
    for (auto* n : closed) delete n;
    return {};
}
bool AStarPlanner::inBounds(const Vec2i& p) const {
    return p.x >= 0 && p.x < size_x_ && p.y >= 0 && p.y < size_y_;
}

bool AStarPlanner::isObstacle(const Vec2i& p) const {
    return costmap_[p.y * size_x_ + p.x] > 100;
}

unsigned int AStarPlanner::calculateCost(const Vec2i& current, const Vec2i& next) const {
    const unsigned int base = 10;
    const double factor = 2.0;
    unsigned int cell_cost = costmap_[next.y * size_x_ + next.x];
    return base + static_cast<unsigned int>(cell_cost * factor);
}

// ===== 휴리스틱 구현 =====
unsigned int AStarPlanner::manhattan(const Vec2i& a, const Vec2i& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

unsigned int AStarPlanner::euclidean(const Vec2i& a, const Vec2i& b) {
    int dx = a.x - b.x, dy = a.y - b.y;
    return static_cast<unsigned int>(std::sqrt(dx * dx + dy * dy));
}

unsigned int AStarPlanner::octagonal(const Vec2i& a, const Vec2i& b) {
    int dx = std::abs(a.x - b.x), dy = std::abs(a.y - b.y);
    return (dx + dy) - 0.586 * std::min(dx, dy);
}
