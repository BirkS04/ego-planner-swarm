#ifndef _GRAPH_NODE_H_
#define _GRAPH_NODE_H_

#include <Eigen/Eigen>
#include <vector>

using Eigen::Vector3d;
using std::vector;

namespace fast_planner {

class ViewNode {
public:
  // Berechnet vereinfachte Kosten (Luftlinie + Yaw-Differenz)
  static double computeCost(const Vector3d& p1, const Vector3d& p2, const double& y1, const double& y2,
                            const Vector3d& v1, const double& yd1, vector<Vector3d>& path);
  
  static void searchPath(const Vector3d& p1, const Vector3d& p2, vector<Vector3d>& path);
};

}  // namespace fast_planner
#endif