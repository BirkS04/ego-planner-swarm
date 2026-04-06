#ifndef _EXPLORATION_MANAGER_H_
#define _EXPLORATION_MANAGER_H_

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <memory>
#include <vector>

// GridMap vom EgoPlanner
#include <plan_env/grid_map.h>
#include <visualization_msgs/msg/marker_array.hpp>

using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class FrontierFinder;
struct ExplorationParam;
struct ExplorationData;

enum EXPL_RESULT { NO_FRONTIER, FAIL, SUCCEED };

class FastExplorationManager {
public:
  FastExplorationManager();
  ~FastExplorationManager();

  // ACHTUNG: ego_planner:: vor GridMap entfernt, da GridMap global ist!
  void initialize(rclcpp::Node::SharedPtr& node, GridMap::Ptr grid_map);

  int getNextExplorationGoal(const Vector3d& pos, const Vector3d& vel, const Vector3d& yaw,
                             Vector3d& next_pos, double& next_yaw);

  shared_ptr<ExplorationData> ed_;
  shared_ptr<ExplorationParam> ep_;
  shared_ptr<FrontierFinder> frontier_finder_;

private:
  rclcpp::Node::SharedPtr node_;
  GridMap::Ptr grid_map_;

  // ---> NEU: Publisher und Methode für RViz
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  void visualizeFrontiers();

  // LKH TSP Solver für die globale Tour
  void findGlobalTour(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw,
                      vector<int>& indices);

  // ==========================================
  // ---> NEU: Tour Memory (TSP Gedächtnis)
  // ==========================================
  std::vector<int> tour_indices_;
  int current_tour_idx_{0};
  rclcpp::Time last_global_plan_time_;

public:
  typedef shared_ptr<FastExplorationManager> Ptr;
};

}  // namespace fast_planner

#endif