#include "nav_keti/a_star.h"
#include <iostream>

using namespace std::placeholders;

bool AStar::Vec2i::operator == (const Vec2i& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
}

bool AStar::Vec2i::operator != (const Vec2i& coordinates_)
{
    return !(x == coordinates_.x && y == coordinates_.y);
}

AStar::Vec2i operator + (const AStar::Vec2i& left_, const AStar::Vec2i& right_)
{
    return { left_.x + right_.x, left_.y + right_.y };
}

AStar::Node::Node(Vec2i coordinates_, Node *parent_)
{
    parent = parent_;
    coordinates = coordinates_;
    G = H = 0;
}

AStar::uint AStar::Node::getScore()
{
    return G + H;
}

AStar::Generator::Generator()
{
    setDiagonalMovement(false);
    setHeuristic(&Heuristic::manhattan);
    direction = {
        { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
        { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }
    };
}

void AStar::Generator::setWorldSize(Vec2i worldSize_)
{
    worldSize = worldSize_;
}

void AStar::Generator::setDiagonalMovement(bool enable_)
{
    directions = (enable_ ? 8 : 4);
}

void AStar::Generator::setHeuristic(HeuristicFunction heuristic_)
{
    heuristic = std::bind(heuristic_, _1, _2);
}

void AStar::Generator::addCollision(Vec2i coordinates_)
{
    if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
        coordinates_.y < 0 || coordinates_.y >= worldSize.y)
        return;

    if(!detectCollision(coordinates_))
        walls.push_back(coordinates_);
}

void AStar::Generator::addCollision(Vec2i coordinates_, int size)
{
    if(size == 0)
    {
        addCollision(coordinates_);
        return;
    }

    for(int i=0; i<4; i++)
    {
        Vec2i new_coord = coordinates_ + direction[i];
        addCollision(new_coord, size-1);
    }
}

void AStar::Generator::removeCollision(Vec2i coordinates_) 
{
    auto it = std::find(walls.begin(), walls.end(), coordinates_);
    if (it != walls.end()) {
        walls.erase(it);
    }
}

void AStar::Generator::clearCollisions() {

    walls.clear();
}

void AStar::Generator::setCostmap(const std::vector<uint8_t>& costmap_data) 
{
    costmap_ = costmap_data;
}

// uint AStar::Generator::calculateCost(Vec2i current, Vec2i next) {
//     int index = next.y * worldSize.x + next.x;
//     uint cell_cost = (index < costmap_.size()) ? costmap_[index] : 0;
//     uint base_cost = (current.x == next.x || current.y == next.y) ? 10 : 14;
//     return base_cost + (cell_cost * 2);
// }

// uint AStar::Generator::calculateCost(Vec2i current, Vec2i next) {

//     const uint threshold = 250;  
//     const uint base_value = 5;  
//     const double scaling_factor = 0.15; 
//     const uint diagonal_factor = 1.5;  

//     int index = next.y * worldSize.x + next.x;
//     uint cell_cost = costmap_[index];

//     uint base_cost = (current.x == next.x || current.y == next.y) ? base_value : base_value * diagonal_factor;

//     if (cell_cost >= threshold) {
//         // return base_cost + static_cast<uint>(cell_cost * scaling_factor) + 1000;
//         return base_cost + static_cast<uint>(cell_cost * 5);
//     }
//     // if (cell_cost == 255) {
//     //     return base_cost; 
//     // }
//     return base_cost + static_cast<uint>(cell_cost * scaling_factor);
// }

uint AStar::Generator::calculateCost(Vec2i current, Vec2i next) {

    const uint base_value = 5;  //기본 비용, 높으면 장애물 무시함
    const double scaling_factor = 0.1; //장애물을 얼마나 무시할지
    const uint diagonal_factor = 1.414;  //대각선 비용

    int index = next.y * worldSize.x + next.x;
    uint cell_cost = costmap_[index];

    uint base_cost = (current.x == next.x || current.y == next.y) ? base_value : base_value * diagonal_factor;

    if (cell_cost == 255) {
        return base_cost; 
    }

    return base_cost + static_cast<uint>(cell_cost * scaling_factor);
}

// uint AStar::Generator::calculateCost(Vec2i current, Vec2i next) {

//     const uint threshold = 250;  
//     const uint base_value = 5;
//     const uint diagonal_factor = 1.5;

//     int index = next.y * worldSize.x + next.x;
//     uint base_cost = (current.x == next.x || current.y == next.y) ? base_value : base_value * diagonal_factor;

//     uint cell_cost = costmap_[index];
//     if (cell_cost < threshold) {
//         return base_cost + cell_cost; 
//     }

//     return base_cost + cell_cost * 5;
// }


// AStar::CoordinateList AStar::Generator::findPath(Vec2i source_, Vec2i target_)
// {
//     Node *current = nullptr;
//     NodeSet openSet, closedSet;
//     openSet.reserve(100);
//     closedSet.reserve(100);
//     openSet.push_back(new Node(source_));

//     struct timespec start, end;
//     clock_gettime(CLOCK_MONOTONIC, &start);

//     while (!openSet.empty()) {
//         auto current_it = openSet.begin();
//         current = *current_it;

//         for (auto it = openSet.begin(); it != openSet.end(); it++) {
//             auto node = *it;
//             if (node->getScore() <= current->getScore()) {
//                 current = node;
//                 current_it = it;
//             }
//         }

//         clock_gettime(CLOCK_MONOTONIC, &end);
//         double time_result = end.tv_sec - start.tv_sec + (end.tv_nsec - start.tv_nsec)*1e-9;

//         if (time_result > 1.0) {
//             printf("Pathfinding exceeded time limit: %lf\n", time_result);
//             break;
//         }

//         if (current->coordinates == target_) {
//             break;
//         }

//         closedSet.push_back(current);
//         openSet.erase(current_it);

//         for (uint i = 0; i < directions; ++i) {
//             Vec2i newCoordinates(current->coordinates + direction[i]);
//             if (detectCollision(newCoordinates) || findNodeOnList(closedSet, newCoordinates)) {
//                 continue;
//             }

//             uint totalCost = current->G + calculateCost(current->coordinates, newCoordinates);

//             Node *successor = findNodeOnList(openSet, newCoordinates);
//             if (successor == nullptr) {
//                 successor = new Node(newCoordinates, current);
//                 successor->G = totalCost;
//                 successor->H = heuristic(successor->coordinates, target_);
//                 openSet.push_back(successor);
//             }
//             else if (totalCost < successor->G) {
//                 successor->parent = current;
//                 successor->G = totalCost;
//             }
//         }
//     }

//     CoordinateList path;
//     if (current->coordinates == target_) {
//         while (current != nullptr) {
//             path.push_back(current->coordinates);
//             current = current->parent;
//         }
//     }

//     releaseNodes(openSet);
//     releaseNodes(closedSet);
//     return path;
// }

AStar::CoordinateList AStar::Generator::findPath(Vec2i source_, Vec2i target_)
{
    Node *current = nullptr;
    NodeSet openSet, closedSet;
    openSet.reserve(100);
    closedSet.reserve(100);
    openSet.push_back(new Node(source_));

    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);

    CoordinateList bestPath;
    
    while (!openSet.empty()) {
        auto current_it = openSet.begin();
        current = *current_it;

        for (auto it = openSet.begin(); it != openSet.end(); it++) {
            auto node = *it;
            if (node->getScore() <= current->getScore()) {
                current = node;
                current_it = it;
            }
        }

        clock_gettime(CLOCK_MONOTONIC, &end);
        double time_result = end.tv_sec - start.tv_sec + (end.tv_nsec - start.tv_nsec) * 1e-9;

        if (time_result > 1.0) {
            printf("Pathfinding exceeded time limit: %lf\n", time_result);
            break;  // 제한 시간 초과 시 루프 종료
        }

        if (current->coordinates == target_) {
            break;
        }

        closedSet.push_back(current);
        openSet.erase(current_it);

        for (uint i = 0; i < directions; ++i) {
            Vec2i newCoordinates(current->coordinates + direction[i]);
            if (detectCollision(newCoordinates) || findNodeOnList(closedSet, newCoordinates)) {
                continue;
            }

            uint totalCost = current->G + calculateCost(current->coordinates, newCoordinates);

            Node *successor = findNodeOnList(openSet, newCoordinates);

            if (successor == nullptr) {
                successor = new Node(newCoordinates, current);
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_) ;
                openSet.push_back(successor);
            }
            else if (totalCost < successor->G) {
                successor->parent = current;
                successor->G = totalCost;
            }
        }
    }

    // 경로 추적
    while (current != nullptr) {
        bestPath.push_back(current->coordinates);
        current = current->parent;
    }

    releaseNodes(openSet);
    releaseNodes(closedSet);
    
    if (bestPath.empty()) {
        printf("No valid path found within the time limit.\n");
    }

    return bestPath;  // 제한 시간 내 탐색된 최적의 경로 반환
}

AStar::Node* AStar::Generator::findNodeOnList(NodeSet& nodes_, Vec2i coordinates_)
{
    for (auto node : nodes_) {
        if (node->coordinates == coordinates_) {
            return node;
        }
    }
    return nullptr;
}

void AStar::Generator::releaseNodes(NodeSet& nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete *it;
        it = nodes_.erase(it);
    }
}

bool AStar::Generator::detectCollision(Vec2i coordinates_)
{
    if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
        coordinates_.y < 0 || coordinates_.y >= worldSize.y ||
        std::find(walls.begin(), walls.end(), coordinates_) != walls.end())
    {
        return true;
    }

    return false;
}

AStar::Vec2i AStar::Heuristic::getDelta(Vec2i source_, Vec2i target_)
{
    return{ abs(source_.x - target_.x),  abs(source_.y - target_.y) };
}

AStar::uint AStar::Heuristic::manhattan(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * (delta.x + delta.y));
}

AStar::uint AStar::Heuristic::euclidean(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

AStar::uint AStar::Heuristic::octagonal(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}
