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

uint AStar::Generator::calculateCost(Vec2i current, Vec2i next) {
    const uint base_value = 5;  //기본 비용
    const double scaling_factor = 2.5; //장애물을 얼마나 무시할지, 낮으면 무시함
    const double diagonal_factor = 1.414;  //대각선 비용
    int index = next.y * worldSize.x + next.x;
    uint cell_cost = costmap_[index];

    uint base_cost = (current.x == next.x || current.y == next.y) ? base_value : base_value * diagonal_factor;
    //std::cout<<static_cast<uint>(cell_cost * scaling_factor)<<std::endl;
    return base_cost + static_cast<uint>(cell_cost * scaling_factor); //G값은 uint로 정의되어 있어 이렇게 적절하게 사용해야 
}

AStar::CoordinateList AStar::Generator::findPath(Vec2i source_, Vec2i target_) {
    Node* current = new Node(source_);
    NodeSet closedSet;
    CoordinateList bestPath;

    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);

    // 탐색 루프 시작
    while (true) {
        // 시간 초과 검사
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

            // 충돌이 있거나 이미 방문한 노드는 무시
            if (detectCollision(neighborCoordinates) || findNodeOnList(closedSet, neighborCoordinates)) {
                continue;
            }

            // 이웃 노드의 F 값 계산
            uint neighborG = current->G + calculateCost(current->coordinates, neighborCoordinates)* 5;
            uint neighborH = static_cast<uint>(10.0 * heuristic(neighborCoordinates, target_));
            uint neighborF = neighborG + neighborH;

            // F 값이 가장 작은 노드 선택
            if (neighborF < minF) {
                minF = neighborF;

                // 이웃 노드가 openSet에 없으면 새로 생성
                Node* successor = new Node(neighborCoordinates, current);
                successor->G = neighborG;
                successor->H = neighborH;

                bestNeighbor = successor;
            }
        }

        // 더 이상 이동할 노드가 없으면 탐색 종료
        if (bestNeighbor == nullptr) {
            printf("No valid path found.\n");
            break;
        }

        // 현재 노드를 closedSet에 추가하고 다음 노드로 이동
        closedSet.push_back(current);
        current = bestNeighbor;
    }

    // 경로 추적
    while (current != nullptr) {
        bestPath.push_back(current->coordinates);
        current = current->parent;
    }

    // if (!bestPath.empty()) {
    //     std::cout << "출발합니다";
    //     for (auto it = bestPath.rbegin(); it != bestPath.rend(); ++it) {
    //         std::cout << "(" << it->x << ", " << it->y << ") -> ";
    //     }
    //     std::cout << "도착" << std::endl;
    // } else {
    //     std::cout << "경로를 찾을 수 없습니다." << std::endl;
    // }
    // 메모리 해제
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

AStar::uint AStar::Heuristic::manhattan(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>((delta.x + delta.y));
}

AStar::uint AStar::Heuristic::euclidean(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

AStar::uint AStar::Heuristic::octagonal(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
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
