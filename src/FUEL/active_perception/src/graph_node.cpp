#include <active_perception/graph_node.h>
#include <cmath>

namespace fast_planner {

double ViewNode::computeCost(const Vector3d& p1, const Vector3d& p2, const double& y1, const double& y2,
                             const Vector3d& v1, const double& yd1, vector<Vector3d>& path) {

  (void)v1;   // <--- NEU (Unterdrückt die Warnung)
  (void)yd1;  // <--- NEU (Unterdrückt die Warnung)
  
  // Euklidische Distanz (Luftlinie)
  double dist = (p1 - p2).norm();
  
  // Yaw-Differenz berechnen
  double dyaw = fabs(y1 - y2);
  if (dyaw > M_PI) dyaw = 2 * M_PI - dyaw;
  
  // Wir geben einfach eine gerade Linie als Platzhalter zurück.
  // EgoPlanner fliegt später ohnehin seine eigene, sichere B-Spline Route!
  path.clear();
  path.push_back(p1);
  path.push_back(p2);
  
  // Kosten = Distanz + (Gewichtung des Drehwinkels)
  return dist + dyaw * 0.5; 
}

void ViewNode::searchPath(const Vector3d& p1, const Vector3d& p2, vector<Vector3d>& path) {
  path.clear();
  path.push_back(p1);
  path.push_back(p2);
}

} // namespace fast_planner