#ifndef RMP_PATH_PLANNER_SAMPLE_PLANNER_RRT_H_
#define RMP_PATH_PLANNER_SAMPLE_PLANNER_RRT_H_

#include <vector>
#include <unordered_map>
#include <cstdint>
#include "collision_checker.h"
#include "node.h"
#include "point.h"
#include <rclcpp/rclcpp.hpp>
namespace rmp
{
namespace path_planner
{

class RRTPathPlanner
{
private:
  using Node = rmp::common::structure::Node<int>;

public:
  struct CostmapInfo {
    int size_x, size_y;
    double resolution;
    double origin_x, origin_y;
    std::vector<uint8_t> data;
  };

  RRTPathPlanner(double obstacle_factor, int sample_num, double max_dist);
  void setCostmapData(const CostmapInfo& info);

  bool plan(const rmp::common::geometry::Point3d& start,
            const rmp::common::geometry::Point3d& goal,
            std::vector<rmp::common::geometry::Point3d>& path,
            std::vector<rmp::common::geometry::Point3d>& expand);

private:
  Node _generateRandomNode();
  Node _findNearestPoint(std::unordered_map<int, Node>& list, const Node& node);
  bool _checkGoal(const Node& node);
  bool isCollision(const Node& n1, const Node& n2) const;

  int grid2Index(int x, int y) const;
  void index2Grid(int idx, int& x, int& y) const;
  void mapToWorld(int mx, int my, double& wx, double& wy) const;

private:
  double obstacle_factor_;
  int sample_num_;
  double max_dist_;
  double opti_sample_p_ = 0.05;

  int size_x_, size_y_, map_size_;
  double resolution_, origin_x_, origin_y_;
  std::vector<uint8_t> costmap_;

  Node start_, goal_;
  std::unordered_map<int, Node> sample_list_;
};

}  // namespace path_planner
}  // namespace rmp

#endif
