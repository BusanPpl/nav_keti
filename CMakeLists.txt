cmake_minimum_required(VERSION 3.5)
project(nav_keti)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GeographicLib REQUIRED)

pkg_check_modules(PROJ REQUIRED proj)

include_directories(${PROJ_INCLUDE_DIRS})
link_directories(${PROJ_LIBRARY_DIRS})


include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)

add_executable(costmap_tracking src/costmap_generator.cpp src/obstacle_tracking.cpp src/dynamic_costmap_updater.cpp src/operation_costmap_updater.cpp)
ament_target_dependencies(costmap_tracking rclcpp std_msgs nav_msgs sensor_msgs tf2_ros visualization_msgs nav2_msgs nlohmann_json OpenCV GeographicLib) 

add_executable(global_path src/global_path_planner_node.cpp src/rrt_star_planner.cpp src/astar_planner.cpp)
ament_target_dependencies(global_path rclcpp std_msgs nav_msgs sensor_msgs tf2_ros visualization_msgs nav2_msgs GeographicLib) 
target_link_libraries(global_path ${PROJ_LIBRARIES})

add_executable(local_path src/local_path_plan.cpp src/dwa.cpp)
ament_target_dependencies(local_path rclcpp std_msgs nav_msgs sensor_msgs tf2_ros visualization_msgs nav2_msgs GeographicLib) 

add_executable(yaw src/yaw_test.cpp)
ament_target_dependencies(yaw rclcpp std_msgs nav_msgs sensor_msgs tf2_ros visualization_msgs nav2_msgs GeographicLib) 

install(TARGETS
  costmap_tracking
  local_path
  global_path
  yaw
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
