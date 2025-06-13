#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include <cstdint>
#include <string>
#include <memory>

struct Vec2i {
    int x, y;
    Vec2i(int x_ = 0, int y_ = 0) : x(x_), y(y_) {}
    bool operator==(const Vec2i& o) const { return x == o.x && y == o.y; }
};

struct Node {
    Vec2i pos;
    Node* parent = nullptr;
    unsigned int g = 0, h = 0;
    double cost = 0.0;
    std::vector<Vec2i> path;
    Node(Vec2i p, Node* parent_ = nullptr) : pos(p), parent(parent_) {}
    unsigned int f() const { return g + h; }
};

class PathPlannerInterface {
    public:
    //오버라이드 안해도 무방
        virtual void setStart(const Vec2i& start) {
            start_ = start;
        }
    
        virtual void setGoal(const Vec2i& goal) {
            goal_ = goal;
        }
    
        virtual void setCostmapData(const std::vector<uint8_t>& data, int size_x, int size_y, double resolution) {
            costmap_ = data;
            size_x_ = size_x;
            size_y_ = size_y;
            resolution_ = resolution;
        }
    
        virtual std::vector<Vec2i> plan() = 0; // 무조건 오버라이드 해야함
        virtual ~PathPlannerInterface() = default;
        virtual std::vector<Vec2i> smoothPath(const std::vector<Vec2i>& path, int max_iter) {
            return path;
        }
    protected:
        Vec2i start_, goal_;
        std::vector<uint8_t> costmap_;
        int size_x_{0}, size_y_{0};
        double resolution_{0.05};
    };

#endif // PATH_PLANNER_H
