#include "nav_keti/astar_v2.h"
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

double AStar::Node::getScore()
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

void AStar::Generator::removeCollision(Vec2i coordinates_) {
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
    addCollisionsFromCostmap(200);
}

void AStar::Generator::addCollisionsFromCostmap(int threshold) {
    walls.clear();  // 이전 장애물 정보 초기화

    for (int y = 0; y < worldSize.y; ++y) {
        for (int x = 0; x < worldSize.x; ++x) {
            int index = y * worldSize.x + x;

            // 셀 값이 임계값 이상인 경우 장애물로 간주
            if (costmap_[index] >= threshold) {
                walls.push_back({x, y});
            }
        }
    }
}

double AStar::Generator::calculateCost(Vec2i current, Vec2i next) {
    double base_cost = std::hypot(current.x - next.x, current.y - next.y);

    int index = next.y * worldSize.x + next.x;
    double cell_cost = 26 + 0.9 * static_cast<double>(costmap_[index]); //[26, 255)
    double w_traversal_cost_ = 2.0; 
    return base_cost + w_traversal_cost_ * cell_cost * cell_cost / std::pow(255, 2);
}

AStar::CoordinateList AStar::Generator::findPath(Vec2i source_, Vec2i target_) {
    Node* current = new Node(source_);
    NodeSet closedSet;
    CoordinateList bestPath;

    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);

    while (true) {
        clock_gettime(CLOCK_MONOTONIC, &end);
        double time_result = end.tv_sec - start.tv_sec + (end.tv_nsec - start.tv_nsec) * 1e-9;
        if (time_result > 1.0) {
            printf("Pathfinding exceeded time limit: %lf\n", time_result);
            break;
        }

        if (current->coordinates == target_) {
            break;
        }

        Node* bestNeighbor = nullptr;
        uint minF = std::numeric_limits<uint>::max();

        for (uint i = 0; i < directions; ++i) {
            Vec2i neighborCoordinates = current->coordinates + direction[i];

            if (detectCollision(neighborCoordinates) || findNodeOnList(closedSet, neighborCoordinates)) {
                continue;
            }

            Node* optimizedParent = current;
            if (direction[i].x != 0 && direction[i].y != 0 && current->parent != nullptr) {
                if (losCheck(current->parent->coordinates, neighborCoordinates, 1)) {
                    optimizedParent = current->parent;
                }
            }

            uint neighborG = optimizedParent->G + calculateCost(optimizedParent->coordinates, neighborCoordinates);
            uint neighborH = heuristic(neighborCoordinates, target_);
            uint neighborF = neighborG + neighborH;

            if (neighborF < minF) {
                minF = neighborF;

                Node* successor = new Node(neighborCoordinates, optimizedParent);
                successor->G = neighborG;
                successor->H = neighborH;
                bestNeighbor = successor;
            }
        }

        if (bestNeighbor == nullptr) {
            printf("No valid path found.\n");
            break;
        }

        closedSet.push_back(current);
        current = bestNeighbor;
    }

    while (current != nullptr) {
        bestPath.push_back(current->coordinates);
        current = current->parent;
    }

    releaseNodes(closedSet);
    return bestPath;
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

double AStar::Heuristic::manhattan(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<double>((delta.x + delta.y));
}

double AStar::Heuristic::euclidean(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<double>(sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

double AStar::Heuristic::octagonal(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}
bool AStar::Generator::losCheck(Vec2i start, Vec2i end, int allowed_obstacles) {
    int x0 = start.x;
    int y0 = start.y;
    int x1 = end.x;
    int y1 = end.y;

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    int obstacle_count = 0;  // 허용할 장애물 개수

    while (true) {
        if (detectCollision({x0, y0})) {
            obstacle_count++;
            if (obstacle_count > allowed_obstacles) {
                return false;
            }
        }

        if (x0 == x1 && y0 == y1) {
            return true;
        }

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}