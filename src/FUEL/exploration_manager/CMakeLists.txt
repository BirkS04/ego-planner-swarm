cmake_minimum_required(VERSION 3.8)
project(exploration_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(active_perception REQUIRED)
find_package(lkh_tsp_solver REQUIRED)
find_package(plan_env REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(fast_exploration_manager 
  src/fast_exploration_manager.cpp
)

target_include_directories(fast_exploration_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(fast_exploration_manager
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  active_perception
  lkh_tsp_solver
  plan_env
  pcl_ros
  pcl_msgs
  cv_bridge
)

target_link_libraries(fast_exploration_manager ${PCL_LIBRARIES})

ament_export_dependencies(
  rclcpp
  active_perception
  lkh_tsp_solver
  plan_env
  sensor_msgs
  visualization_msgs
  pcl_ros
  pcl_msgs
  cv_bridge
)
ament_export_targets(fast_exploration_managerExport HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

install(TARGETS fast_exploration_manager
  EXPORT fast_exploration_managerExport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()