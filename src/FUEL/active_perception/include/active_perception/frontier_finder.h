#ifndef _FRONTIER_FINDER_H_
#define _FRONTIER_FINDER_H_

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <list>
#include <utility>

// ROS 2 Visualization
#include <visualization_msgs/msg/marker_array.hpp>

// Das ist die magische Brücke zu EgoPlanner!
#include "plan_env/grid_map.h" 

using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;
using std::list;
using std::pair;

class RayCaster;

namespace fast_planner {
class PerceptionUtils;

// Viewpoint to cover a frontier cluster
struct Viewpoint {
  Vector3d pos_;
  double yaw_;
  int visib_num_;
};

// A frontier cluster, the viewpoints to cover it
struct Frontier {
  vector<Vector3d> cells_;
  vector<Vector3d> filtered_cells_;
  Vector3d average_;
  int id_;
  vector<Viewpoint> viewpoints_;
  Vector3d box_min_, box_max_;
  list<vector<Vector3d>> paths_;
  list<double> costs_;
};

class FrontierFinder {
public:
  // KONSTRUKTOR ANGEPASST: Nutzt jetzt GridMap und rclcpp::Node
  FrontierFinder(const GridMap::Ptr& grid_map, rclcpp::Node::SharedPtr nh);
  ~FrontierFinder();

  void searchFrontiers();
  void computeFrontiersToVisit();

  void getFrontiers(vector<vector<Vector3d>>& clusters);
  void getDormantFrontiers(vector<vector<Vector3d>>& clusters);
  void getFrontierBoxes(vector<pair<Vector3d, Vector3d>>& boxes);
  void getTopViewpointsInfo(const Vector3d& cur_pos, vector<Vector3d>& points, vector<double>& yaws,
                            vector<Vector3d>& averages);
  void getViewpointsInfo(const Vector3d& cur_pos, const vector<int>& ids, const int& view_num,
                         const double& max_decay, vector<vector<Vector3d>>& points,
                         vector<vector<double>>& yaws);
  void updateFrontierCostMatrix();
  void getFullCostMatrix(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw,
                         Eigen::MatrixXd& mat);
  void getPathForTour(const Vector3d& pos, const vector<int>& frontier_ids, vector<Vector3d>& path);

  void setNextFrontier(const int& id);
  bool isFrontierCovered();
  void wrapYaw(double& yaw);

  shared_ptr<PerceptionUtils> percep_utils_;

  // Visualisierung in RViz
  void visualizeFrontiers();

private:
  void splitLargeFrontiers(list<Frontier>& frontiers);
  bool splitHorizontally(const Frontier& frontier, list<Frontier>& splits);
  void mergeFrontiers(Frontier& ftr1, const Frontier& ftr2);
  bool isFrontierChanged(const Frontier& ft);
  bool haveOverlap(const Vector3d& min1, const Vector3d& max1, const Vector3d& min2,
                   const Vector3d& max2);
  void computeFrontierInfo(Frontier& frontier);
  void downsample(const vector<Vector3d>& cluster_in, vector<Vector3d>& cluster_out);
  void sampleViewpoints(Frontier& frontier);

  int countVisibleCells(const Vector3d& pos, const double& yaw, const vector<Vector3d>& cluster);
  bool isNearUnknown(const Vector3d& pos);
  vector<Eigen::Vector3i> sixNeighbors(const Eigen::Vector3i& voxel);
  vector<Eigen::Vector3i> tenNeighbors(const Eigen::Vector3i& voxel);
  vector<Eigen::Vector3i> allNeighbors(const Eigen::Vector3i& voxel);
  
  bool isNeighborUnknown(const Eigen::Vector3i& voxel);

  // Schneller Filter für Lidar-Artefakte
  bool isTrueFrontierVoxel(const Eigen::Vector3i& voxel);

  void expandFrontier(const Eigen::Vector3i& first);

  // Wrapper of Grid map
  int toadr(const Eigen::Vector3i& idx);
  bool knownfree(const Eigen::Vector3i& idx);
  bool inmap(const Eigen::Vector3i& idx);

  Eigen::Vector3i searchClearVoxel(const Eigen::Vector3i& pt);
  bool isInBoxes(const vector<pair<Vector3d, Vector3d>>& boxes, const Eigen::Vector3i& idx);
  bool canBeMerged(const Frontier& ftr1, const Frontier& ftr2);
  void findViewpoints(const Vector3d& sample, const Vector3d& ftr_avg, vector<Viewpoint>& vps);

  // Data
  vector<char> frontier_flag_;
  list<Frontier> frontiers_, dormant_frontiers_, tmp_frontiers_;
  vector<int> removed_ids_;
  list<Frontier>::iterator first_new_ftr_;
  Frontier next_frontier_;

  // Params
  int cluster_min_;
  double cluster_size_xy_, cluster_size_z_;
  double candidate_rmax_, candidate_rmin_, candidate_dphi_, min_candidate_dist_,
      min_candidate_clearance_;
  int down_sample_;
  double min_view_finish_fraction_, resolution_;
  int min_visib_num_, candidate_rnum_;
  

  // Utils
  GridMap::Ptr grid_map_; // <-- HIER IST DIE EGOPLANNER KARTE!
  unique_ptr<RayCaster> raycaster_;
  rclcpp::Node::SharedPtr node_; // ROS2 Node Handle
  
  // Publisher für RViz
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace fast_planner
#endif