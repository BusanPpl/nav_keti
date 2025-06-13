#include <vector>
#include <cmath>
#include <queue>
#include <functional>

struct Vec2i {
    int x, y;

    bool operator == (const Vec2i& coordinates_) const {
        return (x == coordinates_.x && y == coordinates_.y);
    }

    bool operator != (const Vec2i& coordinates_) const {
        return !(*this == coordinates_);
    }
};

using CoordinateList = std::vector<Vec2i>;

class ThetaStar {
public:
    ThetaStar(const std::vector<std::vector<int>>& costmap, int width, int height) : costmap_(costmap), width_(width), height_(height) {}

    CoordinateList findPath(const Vec2i& start, const Vec2i& goal) {
        std::priority_queue<Node, std::vector<Node>, CompareNode> openSet;
        std::vector<std::vector<bool>> closedSet(height_, std::vector<bool>(width_, false));

        openSet.push(Node(start, nullptr, 0, heuristic(start, goal)));

        while (!openSet.empty()) {
            Node current = openSet.top();
            openSet.pop();

            if (current.coordinates == goal) {
                return constructPath(current);
            }

            closedSet[current.coordinates.y][current.coordinates.x] = true;

            for (const auto& neighbor : getNeighbors(current.coordinates)) {
                if (isValid(neighbor) && !closedSet[neighbor.y][neighbor.x]) {
                    double gCost = current.g + getTraversalCost(current.coordinates, neighbor);
                    double hCost = heuristic(neighbor, goal);

                    openSet.push(Node(neighbor, new Node(current), gCost, gCost + hCost));
                }
            }
        }

        return {}; // Return an empty path if no path is found
    }

private:
    struct Node {
        Vec2i coordinates;
        Node* parent;
        double g, f;

        Node(const Vec2i& coord, Node* parent_, double gCost, double fCost) : coordinates(coord), parent(parent_), g(gCost), f(fCost) {}
    };

    struct CompareNode {
        bool operator()(const Node& a, const Node& b) {
            return a.f > b.f;
        }
    };

    std::vector<std::vector<int>> costmap_;
    int width_, height_;

    CoordinateList getNeighbors(const Vec2i& node) {
        CoordinateList neighbors;
        std::vector<Vec2i> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

        for (const auto& dir : directions) {
            Vec2i neighbor = {node.x + dir.x, node.y + dir.y};
            neighbors.push_back(neighbor);
        }

        return neighbors;
    }

    bool isValid(const Vec2i& point) {
        return (point.x >= 0 && point.x < width_ && point.y >= 0 && point.y < height_ && costmap_[point.y][point.x] < 255);
    }

    double getTraversalCost(const Vec2i& current, const Vec2i& next) {
        return 1.0 + costmap_[next.y][next.x] / 255.0;
    }

    double heuristic(const Vec2i& a, const Vec2i& b) {
        return std::hypot(a.x - b.x, a.y - b.y);
    }

    CoordinateList constructPath(const Node& node) {
        CoordinateList path;

        const Node* current = &node;
        while (current != nullptr) {
            path.push_back(current->coordinates);
            current = current->parent;
        }

        std::reverse(path.begin(), path.end());
        return path;
    }
};
