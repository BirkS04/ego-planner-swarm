#include <active_perception/frontier_finder.h>
#include <plan_env/raycast.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <active_perception/perception_utils.h>
#include <active_perception/graph_node.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Eigenvalues>
#include <queue>

using std::queue;
using std::max;
using std::min;
using std::make_pair;

namespace fast_planner {

FrontierFinder::FrontierFinder(const GridMap::Ptr& grid_map, rclcpp::Node::SharedPtr nh) {
  this->grid_map_ = grid_map;
  this->node_ = nh;
  
  int voxel_num = grid_map_->getVoxelNum();
  frontier_flag_ = vector<char>(voxel_num, 0);
  fill(frontier_flag_.begin(), frontier_flag_.end(), 0);

  node_->declare_parameter("frontier.cluster_min", 100);
  node_->declare_parameter("frontier.cluster_size_xy", 2.0);
  node_->declare_parameter("frontier.cluster_size_z", 2.0);
  node_->declare_parameter("frontier.min_candidate_dist", 1.0);
  node_->declare_parameter("frontier.min_candidate_clearance", 0.5);
  node_->declare_parameter("frontier.candidate_dphi", 0.2);
  node_->declare_parameter("frontier.candidate_rmax", 2.5);
  node_->declare_parameter("frontier.candidate_rmin", 1.0);
  node_->declare_parameter("frontier.candidate_rnum", 3);
  node_->declare_parameter("frontier.down_sample", 2);
  node_->declare_parameter("frontier.min_visib_num", 20);
  node_->declare_parameter("frontier.min_view_finish_fraction", 0.2);
  
  // Schweizer Käse Filter Toleranz
  node_->declare_parameter("frontier.swiss_cheese_tolerance", 25);

  node_->get_parameter("frontier.cluster_min", cluster_min_);
  node_->get_parameter("frontier.cluster_size_xy", cluster_size_xy_);
  node_->get_parameter("frontier.cluster_size_z", cluster_size_z_);
  node_->get_parameter("frontier.min_candidate_dist", min_candidate_dist_);
  node_->get_parameter("frontier.min_candidate_clearance", min_candidate_clearance_);
  node_->get_parameter("frontier.candidate_dphi", candidate_dphi_);
  node_->get_parameter("frontier.candidate_rmax", candidate_rmax_);
  node_->get_parameter("frontier.candidate_rmin", candidate_rmin_);
  node_->get_parameter("frontier.candidate_rnum", candidate_rnum_);
  node_->get_parameter("frontier.down_sample", down_sample_);
  node_->get_parameter("frontier.min_visib_num", min_visib_num_);
  node_->get_parameter("frontier.min_view_finish_fraction", min_view_finish_fraction_);
  node_->get_parameter("frontier.swiss_cheese_tolerance", swiss_cheese_tolerance_);

  raycaster_.reset(new RayCaster);
  resolution_ = grid_map_->getResolution();
  
  percep_utils_.reset(new PerceptionUtils(nh));

  // Publisher für RViz Markers
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("fuel_frontiers", 10);
}

FrontierFinder::~FrontierFinder() {}

void FrontierFinder::searchFrontiers() {
  rclcpp::Time t1 = node_->get_clock()->now();
  tmp_frontiers_.clear();

  Vector3d update_min, update_max, size;
  grid_map_->getRegion(update_min, size);
  update_max = update_min + size;

  auto resetFlag = [&](list<Frontier>::iterator& iter, list<Frontier>& frontiers) {
    Eigen::Vector3i idx;
    for (auto cell : iter->cells_) {
      grid_map_->posToIndex(cell, idx);
      frontier_flag_[toadr(idx)] = 0;
    }
    iter = frontiers.erase(iter);
  };

  removed_ids_.clear();
  int rmv_idx = 0;
  for (auto iter = frontiers_.begin(); iter != frontiers_.end();) {
    if (haveOverlap(iter->box_min_, iter->box_max_, update_min, update_max) && isFrontierChanged(*iter)) {
      resetFlag(iter, frontiers_);
      removed_ids_.push_back(rmv_idx);
    } else {
      ++rmv_idx; ++iter;
    }
  }

  for (auto iter = dormant_frontiers_.begin(); iter != dormant_frontiers_.end();) {
    if (haveOverlap(iter->box_min_, iter->box_max_, update_min, update_max) && isFrontierChanged(*iter))
      resetFlag(iter, dormant_frontiers_);
    else
      ++iter;
  }

  Eigen::Vector3i min_id, max_id;
  grid_map_->posToIndex(update_min, min_id);
  grid_map_->posToIndex(update_max, max_id);

  int count_known_free = 0;
  int count_is_frontier = 0;
  int count_expanded = 0;

  for (int x = min_id(0); x <= max_id(0); ++x) {
    for (int y = min_id(1); y <= max_id(1); ++y) {
      for (int z = min_id(2); z <= max_id(2); ++z) {
        Eigen::Vector3i cur(x, y, z);
        
        if (knownfree(cur)) {
          count_known_free++;
          
          // NEU: Nutze den robusten Volumetrischen Check statt dem einfachen isNeighborUnknown
          if (frontier_flag_[toadr(cur)] == 0 && isTrueFrontierVoxel(cur)) {
            count_is_frontier++;
            expandFrontier(cur);
            count_expanded++;
          }
        }
      }
    }
  }
  
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
    "[FRONTIER DEBUG 1] Map-Scan beendet! knownFree: %d | Frontier-Zellen: %d | expandFrontier() aufgerufen: %d | Valide Cluster in Liste: %lu", 
    count_known_free, count_is_frontier, count_expanded, tmp_frontiers_.size());

  splitLargeFrontiers(tmp_frontiers_);
}


void FrontierFinder::expandFrontier(const Eigen::Vector3i& first) {
  queue<Eigen::Vector3i> cell_queue;
  vector<Eigen::Vector3d> expanded;
  Vector3d pos;

  grid_map_->indexToPos(first, pos);
  expanded.push_back(pos);
  cell_queue.push(first);
  frontier_flag_[toadr(first)] = 1;

  int dropped_by_height = 0;

  while (!cell_queue.empty()) {
    auto cur = cell_queue.front();
    cell_queue.pop();
    auto nbrs = allNeighbors(cur);
    for (auto nbr : nbrs) {
      if (!inmap(nbr)) continue;
      int adr = toadr(nbr);
      
      // NEU: isTrueFrontierVoxel statt isNeighborUnknown
      if (frontier_flag_[adr] == 1 || !(knownfree(nbr) && isTrueFrontierVoxel(nbr)))
        continue;

      grid_map_->indexToPos(nbr, pos);
      
      if (pos[2] < 0.4) {
        dropped_by_height++;
        continue;  
      }
      
      expanded.push_back(pos);
      cell_queue.push(nbr);
      frontier_flag_[adr] = 1;
    }
  }

  if (expanded.size() > 0) {
    if ((int)expanded.size() > cluster_min_) {
      Frontier frontier;
      frontier.cells_ = expanded;
      computeFrontierInfo(frontier);
      tmp_frontiers_.push_back(frontier);
    } else {
      for (auto pt_w : expanded) {
        Eigen::Vector3i idx;
        grid_map_->posToIndex(pt_w, idx);
        frontier_flag_[toadr(idx)] = 0;
      }
      
      RCLCPP_INFO(node_->get_logger(), 
        "[FRONTIER DEBUG 2] Cluster VERWORFEN: Groesse = %lu (Minimum ist %d). Rausgefiltert durch Z-Hoehe: %d", 
        expanded.size(), cluster_min_, dropped_by_height);
    }
  }
}


void FrontierFinder::splitLargeFrontiers(list<Frontier>& frontiers) {
  list<Frontier> splits, tmps;
  for (auto it = frontiers.begin(); it != frontiers.end(); ++it) {
    if (splitHorizontally(*it, splits)) {
      tmps.insert(tmps.end(), splits.begin(), splits.end());
      splits.clear();
    } else
      tmps.push_back(*it);
  }
  frontiers = tmps;
}

bool FrontierFinder::splitHorizontally(const Frontier& frontier, list<Frontier>& splits) {
  auto mean = frontier.average_.head<2>();
  bool need_split = false;
  for (auto cell : frontier.filtered_cells_) {
    if ((cell.head<2>() - mean).norm() > cluster_size_xy_) {
      need_split = true;
      break;
    }
  }
  if (!need_split) return false;

  Eigen::Matrix2d cov;
  cov.setZero();
  for (auto cell : frontier.filtered_cells_) {
    Eigen::Vector2d diff = cell.head<2>() - mean;
    cov += diff * diff.transpose();
  }
  cov /= double(frontier.filtered_cells_.size());

  Eigen::EigenSolver<Eigen::Matrix2d> es(cov);
  auto values = es.eigenvalues().real();
  auto vectors = es.eigenvectors().real();
  int max_idx = 0;
  double max_eigenvalue = -1000000;
  for (int i = 0; i < values.rows(); ++i) {
    if (values[i] > max_eigenvalue) {
      max_idx = i;
      max_eigenvalue = values[i];
    }
  }
  Eigen::Vector2d first_pc = vectors.col(max_idx);

  Frontier ftr1, ftr2;
  for (auto cell : frontier.cells_) {
    if ((cell.head<2>() - mean).dot(first_pc) >= 0)
      ftr1.cells_.push_back(cell);
    else
      ftr2.cells_.push_back(cell);
  }
  computeFrontierInfo(ftr1);
  computeFrontierInfo(ftr2);

  list<Frontier> splits2;
  if (splitHorizontally(ftr1, splits2)) {
    splits.insert(splits.end(), splits2.begin(), splits2.end());
    splits2.clear();
  } else
    splits.push_back(ftr1);

  if (splitHorizontally(ftr2, splits2))
    splits.insert(splits.end(), splits2.begin(), splits2.end());
  else
    splits.push_back(ftr2);

  return true;
}

bool FrontierFinder::isInBoxes(const vector<pair<Vector3d, Vector3d>>& boxes, const Eigen::Vector3i& idx) {
  Vector3d pt;
  grid_map_->indexToPos(idx, pt);
  for (auto box : boxes) {
    bool inbox = true;
    for (int i = 0; i < 3; ++i) {
      inbox = inbox && pt[i] > box.first[i] && pt[i] < box.second[i];
      if (!inbox) break;
    }
    if (inbox) return true;
  }
  return false;
}

void FrontierFinder::updateFrontierCostMatrix() {
  if (!removed_ids_.empty()) {
    for (auto it = frontiers_.begin(); it != first_new_ftr_; ++it) {
      auto cost_iter = it->costs_.begin();
      auto path_iter = it->paths_.begin();
      int iter_idx = 0;
      for (size_t i = 0; i < removed_ids_.size(); ++i) {
        while (iter_idx < removed_ids_[i]) {
          ++cost_iter;
          ++path_iter;
          ++iter_idx;
        }
        cost_iter = it->costs_.erase(cost_iter);
        path_iter = it->paths_.erase(path_iter);
      }
    }
    removed_ids_.clear();
  }

  auto updateCost = [](const list<Frontier>::iterator& it1, const list<Frontier>::iterator& it2) {
    Viewpoint& vui = it1->viewpoints_.front();
    Viewpoint& vuj = it2->viewpoints_.front();
    vector<Vector3d> path_ij;
    double cost_ij = ViewNode::computeCost(vui.pos_, vuj.pos_, vui.yaw_, vuj.yaw_, Vector3d(0, 0, 0), 0, path_ij);
    it1->costs_.push_back(cost_ij);
    it1->paths_.push_back(path_ij);
    reverse(path_ij.begin(), path_ij.end());
    it2->costs_.push_back(cost_ij);
    it2->paths_.push_back(path_ij);
  };

  for (auto it1 = frontiers_.begin(); it1 != first_new_ftr_; ++it1)
    for (auto it2 = first_new_ftr_; it2 != frontiers_.end(); ++it2)
      updateCost(it1, it2);

  for (auto it1 = first_new_ftr_; it1 != frontiers_.end(); ++it1)
    for (auto it2 = it1; it2 != frontiers_.end(); ++it2) {
      if (it1 == it2) {
        it1->costs_.push_back(0);
        it1->paths_.push_back({});
      } else
        updateCost(it1, it2);
    }
}

void FrontierFinder::mergeFrontiers(Frontier& ftr1, const Frontier& ftr2) {
  ftr1.average_ = (ftr1.average_ * double(ftr1.cells_.size()) + ftr2.average_ * double(ftr2.cells_.size())) /
                  (double(ftr1.cells_.size() + ftr2.cells_.size()));
  ftr1.cells_.insert(ftr1.cells_.end(), ftr2.cells_.begin(), ftr2.cells_.end());
  computeFrontierInfo(ftr1);
}

bool FrontierFinder::canBeMerged(const Frontier& ftr1, const Frontier& ftr2) {
  Vector3d merged_avg = (ftr1.average_ * double(ftr1.cells_.size()) + ftr2.average_ * double(ftr2.cells_.size())) /
                        (double(ftr1.cells_.size() + ftr2.cells_.size()));
  for (auto c1 : ftr1.cells_) {
    auto diff = c1 - merged_avg;
    if (diff.head<2>().norm() > cluster_size_xy_ || diff[2] > cluster_size_z_) return false;
  }
  for (auto c2 : ftr2.cells_) {
    auto diff = c2 - merged_avg;
    if (diff.head<2>().norm() > cluster_size_xy_ || diff[2] > cluster_size_z_) return false;
  }
  return true;
}

bool FrontierFinder::haveOverlap(const Vector3d& min1, const Vector3d& max1, const Vector3d& min2, const Vector3d& max2) {
  Vector3d bmin, bmax;
  for (int i = 0; i < 3; ++i) {
    bmin[i] = max(min1[i], min2[i]);
    bmax[i] = min(max1[i], max2[i]);
    if (bmin[i] > bmax[i] + 1e-3) return false;
  }
  return true;
}

bool FrontierFinder::isFrontierChanged(const Frontier& ft) {
  for (auto cell : ft.cells_) {
    Eigen::Vector3i idx;
    grid_map_->posToIndex(cell, idx);
    // NEU: isTrueFrontierVoxel statt isNeighborUnknown
    if (!(knownfree(idx) && isTrueFrontierVoxel(idx))) return true;
  }
  return false;
}

void FrontierFinder::computeFrontierInfo(Frontier& ftr) {
  ftr.average_.setZero();
  ftr.box_max_ = ftr.cells_.front();
  ftr.box_min_ = ftr.cells_.front();
  for (auto cell : ftr.cells_) {
    ftr.average_ += cell;
    for (int i = 0; i < 3; ++i) {
      ftr.box_min_[i] = min(ftr.box_min_[i], cell[i]);
      ftr.box_max_[i] = max(ftr.box_max_[i], cell[i]);
    }
  }
  ftr.average_ /= double(ftr.cells_.size());
  downsample(ftr.cells_, ftr.filtered_cells_);
}

void FrontierFinder::computeFrontiersToVisit() {
  first_new_ftr_ = frontiers_.end();
  for (auto& tmp_ftr : tmp_frontiers_) {
    sampleViewpoints(tmp_ftr);
    if (!tmp_ftr.viewpoints_.empty()) {
      list<Frontier>::iterator inserted = frontiers_.insert(frontiers_.end(), tmp_ftr);
      sort(inserted->viewpoints_.begin(), inserted->viewpoints_.end(),
          [](const Viewpoint& v1, const Viewpoint& v2) { return v1.visib_num_ > v2.visib_num_; });
      if (first_new_ftr_ == frontiers_.end()) first_new_ftr_ = inserted;
    } else {
      dormant_frontiers_.push_back(tmp_ftr);
    }
  }
  int idx = 0;
  for (auto& ft : frontiers_) {
    ft.id_ = idx++;
  }
  
  // RViz Visualisierung aufrufen
  visualizeFrontiers();
}

void FrontierFinder::getTopViewpointsInfo(const Vector3d& cur_pos, vector<Eigen::Vector3d>& points, vector<double>& yaws, vector<Eigen::Vector3d>& averages) {
  points.clear();
  yaws.clear();
  averages.clear();
  for (auto frontier : frontiers_) {
    bool no_view = true;
    for (auto view : frontier.viewpoints_) {
      if ((view.pos_ - cur_pos).norm() < min_candidate_dist_) continue;
      points.push_back(view.pos_);
      yaws.push_back(view.yaw_);
      averages.push_back(frontier.average_);
      no_view = false;
      break;
    }
    if (no_view) {
      auto view = frontier.viewpoints_.front();
      points.push_back(view.pos_);
      yaws.push_back(view.yaw_);
      averages.push_back(frontier.average_);
    }
  }
}

void FrontierFinder::getViewpointsInfo(const Vector3d& cur_pos, const vector<int>& ids, const int& view_num, const double& max_decay, vector<vector<Eigen::Vector3d>>& points, vector<vector<double>>& yaws) {
  points.clear();
  yaws.clear();
  for (auto id : ids) {
    for (auto frontier : frontiers_) {
      if (frontier.id_ == id) {
        vector<Eigen::Vector3d> pts;
        vector<double> ys;
        int visib_thresh = frontier.viewpoints_.front().visib_num_ * max_decay;
        for (auto view : frontier.viewpoints_) {
          if (pts.size() >= view_num || view.visib_num_ <= visib_thresh) break;
          if ((view.pos_ - cur_pos).norm() < min_candidate_dist_) continue;
          pts.push_back(view.pos_);
          ys.push_back(view.yaw_);
        }
        if (pts.empty()) {
          for (auto view : frontier.viewpoints_) {
            if (pts.size() >= view_num || view.visib_num_ <= visib_thresh) break;
            pts.push_back(view.pos_);
            ys.push_back(view.yaw_);
          }
        }
        points.push_back(pts);
        yaws.push_back(ys);
      }
    }
  }
}

void FrontierFinder::getFrontiers(vector<vector<Eigen::Vector3d>>& clusters) {
  clusters.clear();
  for (auto frontier : frontiers_)
    clusters.push_back(frontier.cells_);
}

void FrontierFinder::getDormantFrontiers(vector<vector<Vector3d>>& clusters) {
  clusters.clear();
  for (auto ft : dormant_frontiers_)
    clusters.push_back(ft.cells_);
}

void FrontierFinder::getFrontierBoxes(vector<pair<Eigen::Vector3d, Eigen::Vector3d>>& boxes) {
  boxes.clear();
  for (auto frontier : frontiers_) {
    Vector3d center = (frontier.box_max_ + frontier.box_min_) * 0.5;
    Vector3d scale = frontier.box_max_ - frontier.box_min_;
    boxes.push_back(make_pair(center, scale));
  }
}

void FrontierFinder::getPathForTour(const Vector3d& pos, const vector<int>& frontier_ids, vector<Vector3d>& path) {
  vector<list<Frontier>::iterator> frontier_indexer;
  for (auto it = frontiers_.begin(); it != frontiers_.end(); ++it)
    frontier_indexer.push_back(it);

  vector<Vector3d> segment;
  ViewNode::searchPath(pos, frontier_indexer[frontier_ids[0]]->viewpoints_.front().pos_, segment);
  path.insert(path.end(), segment.begin(), segment.end());

  for (size_t i = 0; i < frontier_ids.size() - 1; ++i) {
    auto path_iter = frontier_indexer[frontier_ids[i]]->paths_.begin();
    int next_idx = frontier_ids[i + 1];
    for (int j = 0; j < next_idx; ++j)
      ++path_iter;
    path.insert(path.end(), path_iter->begin(), path_iter->end());
  }
}

void FrontierFinder::getFullCostMatrix(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw, Eigen::MatrixXd& mat) {
  int dimen = frontiers_.size();
  mat.resize(dimen + 1, dimen + 1);
  int i = 1, j = 1;
  for (auto ftr : frontiers_) {
    for (auto cs : ftr.costs_) {
      mat(i, j++) = cs;
    }
    ++i;
    j = 1;
  }

  mat.leftCols<1>().setZero();
  for (auto ftr : frontiers_) {
    Viewpoint vj = ftr.viewpoints_.front();
    vector<Vector3d> path;
    mat(0, j++) = ViewNode::computeCost(cur_pos, vj.pos_, cur_yaw[0], vj.yaw_, cur_vel, cur_yaw[1], path);
  }
}

void FrontierFinder::findViewpoints(const Vector3d& sample, const Vector3d& ftr_avg, vector<Viewpoint>& vps) {
  if (!grid_map_->isInMap(sample) || grid_map_->getInflateOccupancy(sample) == 1 || isNearUnknown(sample))
    return;

  double vertical_angle_=1.0, ray_length_=5.0; // Defaults
  auto dir = ftr_avg - sample;
  double hc = atan2(dir[1], dir[0]);

  vector<int> slice_gains;
  for (double phi_h = -M_PI_2; phi_h <= M_PI_2 + 1e-3; phi_h += M_PI / 18) {
    int gain = 0;
    for (double phi_v = -vertical_angle_; phi_v <= vertical_angle_; phi_v += vertical_angle_ / 3) {
      Vector3d end;
      end[0] = sample[0] + ray_length_ * cos(phi_v) * cos(hc + phi_h);
      end[1] = sample[1] + ray_length_ * cos(phi_v) * sin(hc + phi_h);
      end[2] = sample[2] + ray_length_ * sin(phi_v);

      Eigen::Vector3d ray_pt;
      Eigen::Vector3i idx;
      raycaster_->setInput(sample, end);
      while (raycaster_->step(ray_pt)) {
        grid_map_->posToIndex(ray_pt, idx);
        if (!grid_map_->isInMap(idx) || grid_map_->getInflateOccupancy(idx) == 1)
          break;
        if (grid_map_->isUnknown(idx)) ++gain;
      }
    }
    slice_gains.push_back(gain);
  }

  vector<pair<double, int>> yaw_gains;
  double right_angle_ = 0.0;
  for (int i = 0; i < 6; ++i) {
    double yaw = hc - M_PI_2 + M_PI / 9.0 * i + right_angle_;
    int gain = 0;
    for (int j = 2 * i; j < 2 * i + 9; ++j) 
      gain += slice_gains[j];
    yaw_gains.push_back(make_pair(yaw, gain));
  }

  vps.clear();
  sort(yaw_gains.begin(), yaw_gains.end(), [](const pair<double, int>& p1, const pair<double, int>& p2) {
    return p1.second > p2.second;
  });
  for (int i = 0; i < 3; ++i) {
    if (yaw_gains[i].second < min_visib_num_) break;
    Viewpoint vp = { sample, yaw_gains[i].first, yaw_gains[i].second };
    wrapYaw(vp.yaw_);
    vps.push_back(vp);
  }
}

void FrontierFinder::sampleViewpoints(Frontier& frontier) {
  int count_total_samples = 0;
  int count_fail_map_occ = 0;
  int count_fail_unknown = 0;
  int count_fail_visib = 0;
  int count_success = 0;

  for (double rc = candidate_rmin_, dr = (candidate_rmax_ - candidate_rmin_) / candidate_rnum_; rc <= candidate_rmax_ + 1e-3; rc += dr) {
    for (double phi = -M_PI; phi < M_PI; phi += candidate_dphi_) {
      count_total_samples++;
      const Vector3d sample_pos = frontier.average_ + rc * Vector3d(cos(phi), sin(phi), 0);

      if (!grid_map_->isInMap(sample_pos) || grid_map_->getInflateOccupancy(sample_pos) == 1) {
        count_fail_map_occ++;
        continue;
      }

      // Prüfen, ob "Unknown" zu nah ist (oft das Problem bei Fast-LIO Raycasting!)
      if (isNearUnknown(sample_pos)) {
        count_fail_unknown++;
        continue;
      }

      auto& cells = frontier.filtered_cells_;
      Eigen::Vector3d ref_dir = (cells.front() - sample_pos).normalized();
      double avg_yaw = 0.0;
      for (size_t i = 1; i < cells.size(); ++i) {
        Eigen::Vector3d dir = (cells[i] - sample_pos).normalized();
        double yaw = acos(dir.dot(ref_dir));
        if (ref_dir.cross(dir)[2] < 0) yaw = -yaw;
        avg_yaw += yaw;
      }
      avg_yaw = avg_yaw / cells.size() + atan2(ref_dir[1], ref_dir[0]);
      wrapYaw(avg_yaw);

      int visib_num = countVisibleCells(sample_pos, avg_yaw, cells);
      if (visib_num > min_visib_num_) {
        Viewpoint vp = { sample_pos, avg_yaw, visib_num };
        frontier.viewpoints_.push_back(vp);
        count_success++;
      } else {
        count_fail_visib++;
      }
    }
  }

  // LOG AUSGABE: Warum wurden Viewpoints abgelehnt?
  RCLCPP_INFO(node_->get_logger(), 
    "[FRONTIER DEBUG 3] Viewpoints gesucht: %d | ERFOLG: %d | Zu nah an OCC/Out-of-Map: %d | Zu nah an UNKNOWN: %d | Zu wenig sichtbare Voxel (<%d): %d",
    count_total_samples, count_success, count_fail_map_occ, count_fail_unknown, min_visib_num_, count_fail_visib);
}

bool FrontierFinder::isFrontierCovered() {
  Vector3d update_min, update_max, size;
  grid_map_->getRegion(update_min, size);
  update_max = update_min + size;

  auto checkChanges = [&](const list<Frontier>& frontiers) {
    for (auto ftr : frontiers) {
      if (!haveOverlap(ftr.box_min_, ftr.box_max_, update_min, update_max)) continue;
      const int change_thresh = min_view_finish_fraction_ * ftr.cells_.size();
      int change_num = 0;
      for (auto cell : ftr.cells_) {
        Eigen::Vector3i idx;
        grid_map_->posToIndex(cell, idx);
        // NEU: isTrueFrontierVoxel statt isNeighborUnknown
        if (!(knownfree(idx) && isTrueFrontierVoxel(idx)) && ++change_num >= change_thresh)
          return true;
      }
    }
    return false;
  };

  if (checkChanges(frontiers_) || checkChanges(dormant_frontiers_)) return true;

  return false;
}

bool FrontierFinder::isNearUnknown(const Eigen::Vector3d& pos) {
  const int vox_num = floor(min_candidate_clearance_ / resolution_);
  int unknown_cnt = 0;
  int total_voxels = 0;

  for (int x = -vox_num; x <= vox_num; ++x) {
    for (int y = -vox_num; y <= vox_num; ++y) {
      for (int z = -1; z <= 1; ++z) {
        total_voxels++;
        
        Eigen::Vector3d vox;
        vox << pos[0] + x * resolution_, pos[1] + y * resolution_, pos[2] + z * resolution_;
        Eigen::Vector3i idx;
        grid_map_->posToIndex(vox, idx);
        
        if (!grid_map_->isInMap(idx) || grid_map_->isUnknown(idx)) {
          unknown_cnt++;
        }
      }
    }
  }

  // 15% Toleranz für Schweizer Käse bei der Viewpoint-Suche
  if (unknown_cnt > (total_voxels * 0.15)) {
    return true;
  }

  return false;
}

int FrontierFinder::countVisibleCells(const Eigen::Vector3d& pos, const double& yaw, const vector<Eigen::Vector3d>& cluster) {
  percep_utils_->setPose(pos, yaw);
  int visib_num = 0;
  Eigen::Vector3i idx;
  Eigen::Vector3d ray_pt;
  for (auto cell : cluster) {
    if (!percep_utils_->insideFOV(cell)) continue;
    
    raycaster_->setInput(cell, pos);
    bool visib = true;
    while (raycaster_->step(ray_pt)) {
        grid_map_->posToIndex(ray_pt, idx);
        if (!grid_map_->isInMap(idx) || grid_map_->getInflateOccupancy(idx) == 1 || grid_map_->isUnknown(idx)) {
            visib = false;
            break;
        }
    }
    if (visib) visib_num += 1;
  }
  return visib_num;
}

void FrontierFinder::downsample(const vector<Eigen::Vector3d>& cluster_in, vector<Eigen::Vector3d>& cluster_out) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudf(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto cell : cluster_in)
    cloud->points.emplace_back(cell[0], cell[1], cell[2]);

  const double leaf_size = grid_map_->getResolution() * down_sample_;
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(leaf_size, leaf_size, leaf_size);
  sor.filter(*cloudf);

  cluster_out.clear();
  for (auto pt : cloudf->points)
    cluster_out.emplace_back(pt.x, pt.y, pt.z);
}

void FrontierFinder::wrapYaw(double& yaw) {
  while (yaw < -M_PI) yaw += 2 * M_PI;
  while (yaw > M_PI) yaw -= 2 * M_PI;
}

Eigen::Vector3i FrontierFinder::searchClearVoxel(const Eigen::Vector3i& pt) {
  queue<Eigen::Vector3i> init_que;
  vector<Eigen::Vector3i> nbrs;
  Eigen::Vector3i cur, start_idx;
  init_que.push(pt);

  while (!init_que.empty()) {
    cur = init_que.front();
    init_que.pop();
    if (knownfree(cur)) {
      start_idx = cur;
      break;
    }
    nbrs = sixNeighbors(cur);
  }
  return start_idx;
}

inline vector<Eigen::Vector3i> FrontierFinder::sixNeighbors(const Eigen::Vector3i& voxel) {
  vector<Eigen::Vector3i> neighbors(6);
  neighbors[0] = voxel - Eigen::Vector3i(1, 0, 0);
  neighbors[1] = voxel + Eigen::Vector3i(1, 0, 0);
  neighbors[2] = voxel - Eigen::Vector3i(0, 1, 0);
  neighbors[3] = voxel + Eigen::Vector3i(0, 1, 0);
  neighbors[4] = voxel - Eigen::Vector3i(0, 0, 1);
  neighbors[5] = voxel + Eigen::Vector3i(0, 0, 1);
  return neighbors;
}

inline vector<Eigen::Vector3i> FrontierFinder::tenNeighbors(const Eigen::Vector3i& voxel) {
  vector<Eigen::Vector3i> neighbors(10);
  Eigen::Vector3i tmp;
  int count = 0;
  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      if (x == 0 && y == 0) continue;
      tmp = voxel + Eigen::Vector3i(x, y, 0);
      neighbors[count++] = tmp;
    }
  }
  neighbors[count++] = tmp - Eigen::Vector3i(0, 0, 1);
  neighbors[count++] = tmp + Eigen::Vector3i(0, 0, 1);
  return neighbors;
}

inline vector<Eigen::Vector3i> FrontierFinder::allNeighbors(const Eigen::Vector3i& voxel) {
  vector<Eigen::Vector3i> neighbors(26);
  Eigen::Vector3i tmp;
  int count = 0;
  for (int x = -1; x <= 1; ++x)
    for (int y = -1; y <= 1; ++y)
      for (int z = -1; z <= 1; ++z) {
        if (x == 0 && y == 0 && z == 0) continue;
        tmp = voxel + Eigen::Vector3i(x, y, z);
        neighbors[count++] = tmp;
      }
  return neighbors;
}

inline bool FrontierFinder::isNeighborUnknown(const Eigen::Vector3i& voxel) {
  auto nbrs = sixNeighbors(voxel);
  for (auto nbr : nbrs) {
    if (inmap(nbr) && grid_map_->isUnknown(nbr)) return true;
  }
  return false;
}

// =========================================================================
// NEU: Volumetrischer Schweizer-Käse-Filter zur Vermeidung falscher Cluster
// =========================================================================
bool FrontierFinder::isTrueFrontierVoxel(const Eigen::Vector3i& voxel) {
  // Erster Check: Gibt es einen direkten Nachbarn? (Schneller Abbruch)
  if (!isNeighborUnknown(voxel)) return false;

  int unknown_count = 0;
  const int search_radius = 2; // Ergibt ein 5x5x5 Gitter = 125 Voxel

  for (int x = -search_radius; x <= search_radius; ++x) {
    for (int y = -search_radius; y <= search_radius; ++y) {
      for (int z = -search_radius; z <= search_radius; ++z) {
        Eigen::Vector3i nbr = voxel + Eigen::Vector3i(x, y, z);
        
        if (!inmap(nbr) || grid_map_->isUnknown(nbr)) {
          unknown_count++;
        }
      }
    }
  }

  // Nur wenn genug "Unknown" Voxel im Block sind, ist es echter leerer Raum
  return unknown_count >= swiss_cheese_tolerance_;
}

inline int FrontierFinder::toadr(const Eigen::Vector3i& idx) {
  return grid_map_->toAddress(idx);
}

inline bool FrontierFinder::knownfree(const Eigen::Vector3i& idx) {
  return inmap(idx) && grid_map_->isKnownFree(idx);
}

inline bool FrontierFinder::inmap(const Eigen::Vector3i& idx) {
  return grid_map_->isInMap(idx);
}

// =========================================================================
// NEU: RViz Visualisierung für aktive/inaktive Frontiers und Viewpoints
// =========================================================================
void FrontierFinder::visualizeFrontiers() {
  // Wenn niemand in RViz zuhört, überspringen, um CPU zu sparen
  if (marker_pub_->get_subscription_count() == 0) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  
  // Alle alten Marker löschen
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  int id = 0;
  auto now = node_->get_clock()->now();

  // 1. Aktive Frontiers
  for (auto& frontier : frontiers_) {
    visualization_msgs::msg::Marker cell_marker;
    cell_marker.header.frame_id = "map"; // Ändern auf "world" / "odom" je nach TF-Tree
    cell_marker.header.stamp = now;
    cell_marker.ns = "frontier_cells";
    cell_marker.id = id++;
    cell_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    cell_marker.action = visualization_msgs::msg::Marker::ADD;
    cell_marker.scale.x = resolution_;
    cell_marker.scale.y = resolution_;
    cell_marker.scale.z = resolution_;
    
    // Aktive Frontiers: Grün
    cell_marker.color.r = 0.0;
    cell_marker.color.g = 0.5;
    cell_marker.color.b = 0.7;
    cell_marker.color.a = 0.2;

    for (const auto& cell : frontier.cells_) {
      geometry_msgs::msg::Point p;
      p.x = cell[0]; p.y = cell[1]; p.z = cell[2];
      cell_marker.points.push_back(p);
    }
    marker_array.markers.push_back(cell_marker);

    // Bester Viewpoint: Roter Pfeil
    if (!frontier.viewpoints_.empty()) {
      auto best_vp = frontier.viewpoints_.front();
      
      visualization_msgs::msg::Marker vp_marker;
      vp_marker.header.frame_id = "map";
      vp_marker.header.stamp = now;
      vp_marker.ns = "frontier_viewpoints";
      vp_marker.id = id++;
      vp_marker.type = visualization_msgs::msg::Marker::ARROW;
      vp_marker.action = visualization_msgs::msg::Marker::ADD;
      
      geometry_msgs::msg::Point p_start, p_end;
      p_start.x = best_vp.pos_[0];
      p_start.y = best_vp.pos_[1];
      p_start.z = best_vp.pos_[2];
      
      p_end.x = best_vp.pos_[0] + 1.0 * cos(best_vp.yaw_);
      p_end.y = best_vp.pos_[1] + 1.0 * sin(best_vp.yaw_);
      p_end.z = best_vp.pos_[2];

      vp_marker.points.push_back(p_start);
      vp_marker.points.push_back(p_end);

      vp_marker.scale.x = 0.1; // Pfeilschaft
      vp_marker.scale.y = 0.2; // Pfeilspitze Breite
      vp_marker.scale.z = 0.3; // Pfeilspitze Länge
      
      vp_marker.color.r = 1.0;
      vp_marker.color.g = 0.0;
      vp_marker.color.b = 0.0;
      vp_marker.color.a = 1.0;

      marker_array.markers.push_back(vp_marker);
    }
  }

  // 2. Ruhende/Inaktive (Dormant) Frontiers
  for (auto& d_frontier : dormant_frontiers_) {
    visualization_msgs::msg::Marker d_marker;
    d_marker.header.frame_id = "map"; 
    d_marker.header.stamp = now;
    d_marker.ns = "dormant_frontiers";
    d_marker.id = id++;
    d_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    d_marker.action = visualization_msgs::msg::Marker::ADD;
    d_marker.scale.x = resolution_;
    d_marker.scale.y = resolution_;
    d_marker.scale.z = resolution_;
    
    // Inaktive Frontiers: Grau/Durchsichtig
    d_marker.color.r = 0.5; d_marker.color.g = 0.5; d_marker.color.b = 0.5; d_marker.color.a = 0.3;

    for (const auto& cell : d_frontier.cells_) {
      geometry_msgs::msg::Point p;
      p.x = cell[0]; p.y = cell[1]; p.z = cell[2];
      d_marker.points.push_back(p);
    }
    marker_array.markers.push_back(d_marker);
  }

  marker_pub_->publish(marker_array);
}

}  // namespace fast_planner