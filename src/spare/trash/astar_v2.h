/*
    Copyright (c) 2015, Damian Barczynski <daan.net@wp.eu>
    Following tool is licensed under the terms and conditions of the ISC license.
    For more information visit https://opensource.org/licenses/ISC.
*/
#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <vector>
#include <functional>
#include <set>
#include <math.h>
#include <algorithm>
namespace AStar
{
    struct Vec2i
    {
        int x, y;

        bool operator == (const Vec2i& coordinates_);
        bool operator != (const Vec2i& coordinates_);
    };

    // using uint = unsigned int;
    using HeuristicFunction = std::function<int(Vec2i, Vec2i)>;
    using CoordinateList = std::vector<Vec2i>;

    struct Node
    {
        double G, H;
        Vec2i coordinates;
        Node *parent;

        Node(Vec2i coord_, Node *parent_ = nullptr);
        double getScore();
    };

    using NodeSet = std::vector<Node*>;

    class Generator
    {
        bool detectCollision(Vec2i coordinates_);
        Node* findNodeOnList(NodeSet& nodes_, Vec2i coordinates_);
        void releaseNodes(NodeSet& nodes_);

    public:
        Generator();
        void setWorldSize(Vec2i worldSize_);
        void setDiagonalMovement(bool enable_);
        void setHeuristic(HeuristicFunction heuristic_);
        CoordinateList findPath(Vec2i source_, Vec2i target_);
        void addCollision(Vec2i coordinates_);
        void addCollision(Vec2i coordinates_, int size);
        void removeCollision(Vec2i coordinates_);
        void clearCollisions();
        void setCostmap(const std::vector<uint8_t>& costmap_data);

    private:
        HeuristicFunction heuristic;
        CoordinateList direction, walls;
        Vec2i worldSize;
        uint directions;
        std::vector<uint8_t> costmap_;
        double calculateCost(Vec2i current, Vec2i next);
        bool losCheck(Vec2i start, Vec2i end, int allowed_obstacles = 1);
        void addCollisionsFromCostmap(int threshold);
    };

    class Heuristic
    {
        static Vec2i getDelta(Vec2i source_, Vec2i target_);

    public:
        static double manhattan(Vec2i source_, Vec2i target_);
        static double euclidean(Vec2i source_, Vec2i target_);
        static double octagonal(Vec2i source_, Vec2i target_);
    };
}

#endif 
