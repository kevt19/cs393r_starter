//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <vector>

#include "eigen3/Eigen/Dense"

#include "vector_map/vector_map.h"

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace navigation {

struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class Navigation {
 public:

   // Constructor
  explicit Navigation(const std::string& map_file, ros::NodeHandle* n);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel);


  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                         double time);

  // Main function called continously from main
  void Run();
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);
  void OneDTOC(PathOption p, Eigen::Vector2f stg);

  void PopulateWallOccupancyGrid();
  std::vector<Eigen::Vector2f> GetNeighborhoodOfWallPoints(const Eigen::Vector2f p0, 
                                                           const Eigen::Vector2f p1,
                                                           float margin);

  Eigen::Vector2f ClosestPointOnLine(const geometry::line2f line_seg, const Eigen::Vector2f point);
  bool ValidatePlan(const std::vector<Eigen::Vector2f> global_plan, const Eigen::Vector2f r_loc, int* closest_waypoint_idx);
  Eigen::Vector2f GetCarrot(std::vector<Eigen::Vector2f> global_plan, const Eigen::Vector2f r_loc, float r_angle, size_t start_waypoint_idx);
  void ObstacleDetector();
  void ObstacleAvoidance();


 private:
  std::vector<Eigen::Vector2f> waypoints;
  // Whether odometry has been initialized.
  bool odom_initialized_;
  // Whether localization has been initialized.
  bool localization_initialized_;
  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;
  // Odometry-reported robot starting location.
  Eigen::Vector2f odom_start_loc_;
  // Odometry-reported robot starting angle.
  float odom_start_angle_;
  // Latest observed point cloud.
  std::vector<Eigen::Vector2f> point_cloud_;

  std::map<std::pair<int, int>, int> obstacle_grid_counts;
  // Create a map of obstacle bins
  std::map<std::pair<int,int>,bool> obstacle_occupancy_grid; 

  std::vector<std::map<std::pair<int, int>, bool>> wall_occupancy_grids;
  std::map<std::pair<int, int>, bool> global_plan_grid;

  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;
  // Map of the environment.
  vector_map::VectorMap map_;


  // Car parameters
  const float ROBOT_WIDTH_;
  const float ROBOT_LENGTH_;
  const float MAX_CLEARANCE_;
  const float WHEEL_BASE_;
  const float MAX_CURVATURE_;
  const float MAX_ACCEL_;
  const float MAX_DEACCL_;
  const float MAX_SHORT_TERM_GOAL_;
  const float STOPPING_DISTANCE_;
  const float MAX_SPEED_;
  const float DELTA_T_;
  const float SYSTEM_LATENCY_;
  const float OBSTACLE_MARGIN_;


  PathOption GetFreePathLength(PathOption p, Eigen::Vector2f stg);
};

}  // namespace navigation

#endif  // NAVIGATION_H
