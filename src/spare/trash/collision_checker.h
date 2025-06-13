#ifndef RMP_COMMON_GEOMETRY_COLLISION_CHECKER_HPP_
#define RMP_COMMON_GEOMETRY_COLLISION_CHECKER_HPP_

#include <vector>
#include <cmath>
#include <cstdint>


namespace rmp
{
namespace common
{
namespace geometry
{

class CollisionChecker
{
public:
  struct CostmapInfo {
    int size_x, size_y;
    double resolution;
    double origin_x, origin_y;
    std::vector<uint8_t> data;
  };

  CollisionChecker(double obstacle_factor = 1.0) : obstacle_factor_(obstacle_factor) {}

  void setCostmap(const CostmapInfo& info) {
    size_x_ = info.size_x;
    size_y_ = info.size_y;
    resolution_ = info.resolution;
    origin_x_ = info.origin_x;
    origin_y_ = info.origin_y;
    costmap_ = info.data;
  }

  bool isInsideMap(int index) const {
    return index >= 0 && index < static_cast<int>(costmap_.size());
  }

  float getCost(int index) const {
    if (!isInsideMap(index)) return 255;
    return costmap_[index];
  }

  bool inCollision(int index) const {
    return isInsideMap(index) && costmap_[index] >= 254 * obstacle_factor_;
  }

  template <typename Point, typename F_is_obs>
  static bool BresenhamCollisionDetection(const Point& pt1, const Point& pt2, F_is_obs func_is_obs)
  {
    int s_x = (pt1.x() == pt2.x()) ? 0 : (pt1.x() - pt2.x()) / std::abs(pt1.x() - pt2.x());
    int s_y = (pt1.y() == pt2.y()) ? 0 : (pt1.y() - pt2.y()) / std::abs(pt1.y() - pt2.y());
    int d_x = std::abs(pt1.x() - pt2.x());
    int d_y = std::abs(pt1.y() - pt2.y());

    if (d_x > d_y)
    {
      int tau = d_y - d_x;
      int x = pt2.x(), y = pt2.y();
      int e = 0;
      while (x != pt1.x())
      {
        if (e * 2 > tau) { x += s_x; e -= d_y; }
        else if (e * 2 < tau) { y += s_y; e += d_x; }
        else { x += s_x; y += s_y; e += d_x - d_y; }

        if (func_is_obs(Point(x, y))) return true;
      }
    }
    else
    {
      int tau = d_x - d_y;
      int x = pt2.x(), y = pt2.y();
      int e = 0;
      while (y != pt1.y())
      {
        if (e * 2 > tau) { y += s_y; e -= d_x; }
        else if (e * 2 < tau) { x += s_x; e += d_y; }
        else { x += s_x; y += s_y; e += d_y - d_x; }

        if (func_is_obs(Point(x, y))) return true;
      }
    }
    return false;
  }

private:
  double obstacle_factor_;
  int size_x_, size_y_;
  double resolution_, origin_x_, origin_y_;
  std::vector<uint8_t> costmap_;
};

}  // namespace geometry
}  // namespace common
}  // namespace rmp

#endif  // RMP_COMMON_GEOMETRY_COLLISION_CHECKER_HPP_
