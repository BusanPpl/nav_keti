#include "nav_keti/rrt_planner.h"

#include <random>
#include <cmath>

template <typename Node>
std::vector<Node> _convertClosedListToPath(std::unordered_map<int, Node>& list, const Node& start, const Node& goal)
{
  std::vector<Node> path;
  Node current = goal;

  while (current.id() != start.id()) {
    path.push_back(current);
    auto it = list.find(current.pid());
    if (it == list.end()) break;
    current = it->second;
  }

  path.push_back(start);
  return path;
}

namespace rmp
{
namespace path_planner
{
using rmp::common::geometry::CollisionChecker;

RRTPathPlanner::RRTPathPlanner(double obstacle_factor, int sample_num, double max_dist)
  : obstacle_factor_(obstacle_factor), sample_num_(sample_num), max_dist_(max_dist) {}

void RRTPathPlanner::setCostmapData(const CostmapInfo& info) {
  size_x_ = info.size_x;
  size_y_ = info.size_y;
  resolution_ = info.resolution;
  origin_x_ = info.origin_x;
  origin_y_ = info.origin_y;
  costmap_ = info.data;
  map_size_ = size_x_ * size_y_;
}

bool RRTPathPlanner::plan(const rmp::common::geometry::Point3d& start,
  const rmp::common::geometry::Point3d& goal,
  std::vector<rmp::common::geometry::Point3d>& path,
  std::vector<rmp::common::geometry::Point3d>& expand)
{
  int sx = static_cast<int>((start.x() - origin_x_) / resolution_);
  int sy = static_cast<int>((start.y() - origin_y_) / resolution_);
  int gx = static_cast<int>((goal.x() - origin_x_) / resolution_);
  int gy = static_cast<int>((goal.y() - origin_y_) / resolution_);

  path.clear();
  expand.clear();
  sample_list_.clear();

  start_.set_x(sx);
  start_.set_y(sy);
  start_.set_id(grid2Index(sx, sy));
  goal_.set_x(gx);
  goal_.set_y(gy);
  goal_.set_id(grid2Index(gx, gy));
  sample_list_[start_.id()] = start_;
  expand.emplace_back(sx, sy, 0);

  int iteration = 0;
  RCLCPP_INFO(rclcpp::get_logger("RRT"), "Entered RRT plan().");

  while (iteration < sample_num_) {
  Node sample_node = _generateRandomNode();
  int sample_id = sample_node.id();

  if (sample_id < 0 || sample_id >= static_cast<int>(costmap_.size())) {
    RCLCPP_ERROR(rclcpp::get_logger("RRT"), "Invalid sample_node id=%d", sample_id);
    iteration++;
    continue;
  }

  if (costmap_[sample_id] >= 254) {
    iteration++;
    continue;
  }

  if (sample_list_.count(sample_node.id())) {
    iteration++;
    continue;
  }

  Node new_node = _findNearestPoint(sample_list_, sample_node);
  if (new_node.id() == -1) {
    iteration++;
    continue;
  }

  sample_list_[new_node.id()] = new_node;
  expand.emplace_back(new_node.x(), new_node.y(), new_node.pid());

  if (_checkGoal(new_node)) {
    RCLCPP_INFO(rclcpp::get_logger("RRT"), "Path found!");
    auto backtrace = _convertClosedListToPath<Node>(sample_list_, start_, goal_);
    for (auto it = backtrace.rbegin(); it != backtrace.rend(); ++it) {
      double wx, wy;
      mapToWorld(it->x(), it->y(), wx, wy);
      path.emplace_back(wx, wy);
    }
    return true;
  }

    iteration++;
  }

  RCLCPP_WARN(rclcpp::get_logger("RRT"), "No path found after %d iterations.", iteration);
  return false;
}

RRTPathPlanner::Node RRTPathPlanner::_generateRandomNode()
{
  std::random_device rd;
  std::mt19937 eng(rd());
  std::uniform_real_distribution<float> p(0, 1);

  if (p(eng) > opti_sample_p_) {
    std::uniform_int_distribution<int> distr(0, map_size_ - 1);
    int id = distr(eng);
    int x = id % size_x_;
    int y = id / size_x_;
    return Node(x, y, 0, 0, id, 0);
  } else {
    return Node(goal_.x(), goal_.y(), 0, 0, goal_.id(), 0);
  }
}

RRTPathPlanner::Node RRTPathPlanner::_findNearestPoint(std::unordered_map<int, Node>& list, const Node& node)
{
  Node nearest_node, new_node(node);
  double min_dist = std::numeric_limits<double>::max();

  for (const auto& p : list) {
    double dist = std::hypot(p.second.x() - node.x(), p.second.y() - node.y());
    if (dist < min_dist) {
      nearest_node = p.second;
      new_node.set_pid(nearest_node.id());
      new_node.set_g(dist + nearest_node.g());
      min_dist = dist;
    }
  }

  if (min_dist > max_dist_) {
    double theta = std::atan2(new_node.y() - nearest_node.y(), new_node.x() - nearest_node.x());
    new_node.set_x(nearest_node.x() + static_cast<int>(max_dist_ * std::cos(theta)));
    new_node.set_y(nearest_node.y() + static_cast<int>(max_dist_ * std::sin(theta)));
    new_node.set_id(grid2Index(new_node.x(), new_node.y()));
    new_node.set_g(max_dist_ + nearest_node.g());
  }

  if (isCollision(new_node, nearest_node)) new_node.set_id(-1);

  return new_node;
}

bool RRTPathPlanner::_checkGoal(const Node& new_node)
{
  double dist = std::hypot(new_node.x() - goal_.x(), new_node.y() - goal_.y());
  if (dist > max_dist_) return false;

  if (!isCollision(new_node, goal_)) {
    Node goal(goal_.x(), goal_.y(), dist + new_node.g(), 0, grid2Index(goal_.x(), goal_.y()), new_node.id());
    sample_list_[goal.id()] = goal;
    return true;
  }
  return false;
}

bool RRTPathPlanner::isCollision(const Node& n1, const Node& n2) const
{
  return CollisionChecker::BresenhamCollisionDetection(n1, n2, [&](const Node& node) {
    int idx = grid2Index(node.x(), node.y());
    if (idx < 0 || idx >= static_cast<int>(costmap_.size())) {
      RCLCPP_WARN(rclcpp::get_logger("RRT"), "isCollision: node out of bounds x=%d y=%d", node.x(), node.y());
      return true; // out-of-bounds는 충돌로 간주
    }
    return costmap_[idx] >= 254;
  });
}

int RRTPathPlanner::grid2Index(int x, int y) const {
  if (x < 0 || x >= size_x_ || y < 0 || y >= size_y_) {
    RCLCPP_ERROR(rclcpp::get_logger("RRT"), "grid2Index out of bounds: x=%d y=%d (size_x=%d size_y=%d)", x, y, size_x_, size_y_);
    return -1;
  }
  return x + size_x_ * y;
}

void RRTPathPlanner::index2Grid(int idx, int& x, int& y) const {
  x = idx % size_x_;
  y = idx / size_x_;
}

void RRTPathPlanner::mapToWorld(int mx, int my, double& wx, double& wy) const {
  wx = origin_x_ + (mx + 0.5) * resolution_;
  wy = origin_y_ + (my + 0.5) * resolution_;
}

}  // namespace path_planner
}  // namespace rmp
