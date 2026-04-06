#include <exploration_manager/fast_exploration_manager.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <lkh_tsp_solver/lkh_interface.h>
#include <active_perception/perception_utils.h>
#include <active_perception/frontier_finder.h>
#include <exploration_manager/expl_data.h>

using namespace Eigen;

namespace fast_planner {

FastExplorationManager::FastExplorationManager() {}

FastExplorationManager::~FastExplorationManager() {}

void FastExplorationManager::initialize(rclcpp::Node::SharedPtr& node, GridMap::Ptr grid_map) {
  node_ = node;
  grid_map_ = grid_map;

  // FrontierFinder mit der Shared Map von EgoPlanner initialisieren
  frontier_finder_.reset(new FrontierFinder(grid_map_, node_));

  ed_.reset(new ExplorationData);
  ep_.reset(new ExplorationParam);

  // ROS 2 Parameter Deklarationen
  node_->declare_parameter("exploration/top_view_num", 15);
  node_->declare_parameter("exploration/max_decay", 0.8);
  node_->declare_parameter("exploration/tsp_dir", string("/tmp"));
  
  node_->get_parameter("exploration/top_view_num", ep_->top_view_num_);
  node_->get_parameter("exploration/max_decay", ep_->max_decay_);
  node_->get_parameter("exploration/tsp_dir", ep_->tsp_dir_);

  // TSP Parameterdatei schreiben
  std::ofstream par_file(ep_->tsp_dir_ + "/single.par");
  par_file << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/single.tsp\n";
  par_file << "GAIN23 = NO\n";
  par_file << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_ << "/single.txt\n";
  par_file << "RUNS = 1\n";

    // ---> NEU: Publisher erstellen
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("exploration/frontiers", 10);
  
 last_global_plan_time_ = node_->now();
  RCLCPP_INFO(node_->get_logger(), "FastExplorationManager erfolgreich initialisiert.");
}

int FastExplorationManager::getNextExplorationGoal(
    const Vector3d& pos, const Vector3d& vel, const Vector3d& yaw, 
    Vector3d& next_pos, double& next_yaw) {
    
  auto now = node_->now();
  double time_since_last_plan = (now - last_global_plan_time_).seconds();

  // =========================================================================
  // SCHRITT 1: Prüfen, ob wir die gespeicherte TSP-Tour nutzen können
  // Bedingung: Es gibt noch Punkte in der Liste UND die Liste ist jünger als 5s.
  // (Nach 5s erzwingen wir ein Replan, weil sich die Map durch den Flug verändert hat).
  // =========================================================================
  if (current_tour_idx_ < tour_indices_.size() && time_since_last_plan < 5.0) {
      int idx = tour_indices_[current_tour_idx_];
      
      // Sicherheits-Check, falls die Arrays aus irgendeinem Grund geleert wurden
      if (idx < ed_->points_.size()) {
          next_pos = ed_->points_[idx];
          next_yaw = ed_->yaws_[idx];
          
          current_tour_idx_++; // Index für das nächste Mal hochzählen
          
          RCLCPP_INFO(node_->get_logger(), "Nutze TSP Tour (Punkt %d von %lu) -> [%.2f, %.2f, %.2f]", 
                      current_tour_idx_, tour_indices_.size(), next_pos(0), next_pos(1), next_pos(2));
          return SUCCEED;
      }
  }

  // =========================================================================
  // SCHRITT 2: FULL REPLAN (Frontiers suchen & neue TSP Tour berechnen)
  // =========================================================================
  RCLCPP_INFO(node_->get_logger(), "Starte Full Replan (Suche Frontiers & TSP)...");
  auto t1 = node_->now();
  ed_->views_.clear();
  ed_->global_tour_.clear();
  tour_indices_.clear();
  current_tour_idx_ = 0;

  frontier_finder_->searchFrontiers();
  frontier_finder_->computeFrontiersToVisit();
  frontier_finder_->getFrontiers(ed_->frontiers_);

  if (ed_->frontiers_.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Keine Frontiers mehr! Exploration beendet.");
    visualizeFrontiers();
    return NO_FRONTIER;
  }
  
  frontier_finder_->getTopViewpointsInfo(pos, ed_->points_, ed_->yaws_, ed_->averages_);

  if (ed_->points_.size() > 1) {
    findGlobalTour(pos, vel, yaw, tour_indices_);
    
    // Nimm das erste Ziel und setze den Index für den nächsten Aufruf auf 1
    int idx = tour_indices_[0];
    next_pos = ed_->points_[idx];
    next_yaw = ed_->yaws_[idx];
    current_tour_idx_ = 1; 
    
  } else if (ed_->points_.size() == 1) {
    next_pos = ed_->points_[0];
    next_yaw = ed_->yaws_[0];
    // Nur ein Ziel, Tour bleibt leer
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Empty destination.");
    return FAIL;
  }

  last_global_plan_time_ = now; // Timer resetten

  double total_time = (node_->now() - t1).seconds();
  RCLCPP_INFO(node_->get_logger(), "Neuer TSP Plan fertig: [%.2f, %.2f, %.2f] (Dauer: %.3fs)", 
              next_pos(0), next_pos(1), next_pos(2), total_time);

  visualizeFrontiers();
  return SUCCEED;
}


void FastExplorationManager::findGlobalTour(
    const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw,
    vector<int>& indices) {
    
  Eigen::MatrixXd cost_mat;
  frontier_finder_->updateFrontierCostMatrix();
  frontier_finder_->getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat);
  const int dimension = cost_mat.rows();

  std::ofstream prob_file(ep_->tsp_dir_ + "/single.tsp");
  string prob_spec = "NAME : single\nTYPE : ATSP\nDIMENSION : " + std::to_string(dimension) +
      "\nEDGE_WEIGHT_TYPE : EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";

  prob_file << prob_spec;
  const int scale = 100;
  
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = cost_mat(i, j) * scale;
      prob_file << int_cost << " ";
    }
    prob_file << "\n";
  }
  prob_file << "EOF";
  prob_file.close();

  solveTSPLKH((ep_->tsp_dir_ + "/single.par").c_str());

  std::ifstream res_file(ep_->tsp_dir_ + "/single.txt");
  string res;
  while (getline(res_file, res)) {
    if (res.compare("TOUR_SECTION") == 0) break;
  }

  while (getline(res_file, res)) {
    int id = std::stoi(res);
    if (id == 1) continue;
    if (id == -1) break;
    indices.push_back(id - 2); 
  }
  res_file.close();
}

void FastExplorationManager::visualizeFrontiers() {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Alte Marker löschen
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = 3; // DELETEALL
    marker_array.markers.push_back(clear_marker);
    
    // Map Resolution für die Würfelgröße holen
    double res = grid_map_->getResolution();

    // Alle Frontiers durchgehen
    for (size_t i = 0; i < ed_->frontiers_.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map"; // Oder "world", je nach deinem TF-Tree
        marker.header.stamp = node_->now();
        marker.ns = "frontiers";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.scale.x = res;
        marker.scale.y = res;
        marker.scale.z = res;

        // Farbe (Zufällige Farben für verschiedene Cluster)
        marker.color.r = (float)rand() / RAND_MAX;
        marker.color.g = (float)rand() / RAND_MAX;
        marker.color.b = 1.0;
        marker.color.a = 0.8;

        for (auto& pt : ed_->frontiers_[i]) {
            geometry_msgs::msg::Point p;
            p.x = pt(0); p.y = pt(1); p.z = pt(2);
            marker.points.push_back(p);
        }
        marker_array.markers.push_back(marker);
    }
    marker_pub_->publish(marker_array);
}



}  // namespace fast_planner