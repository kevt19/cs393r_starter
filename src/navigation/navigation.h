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
#include "latency_compensation.h"

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace navigation {

struct PathOption {
  float curvature = 0;
  float clearance = 10;
  float free_path_length = 100;
  Eigen::Vector2f obstruction = Eigen::Vector2f::Zero();
  Eigen::Vector2f closest_point = Eigen::Vector2f::Zero();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct NavigationParams {
  // frequency
  float dt = .05f;
  // max velocity
  float max_vel = 1.0f;
  // max acceleration
  float max_accel = 4.0f;
  float max_decel = 4.0f;
  // max angular velocity
  float max_omega = 1.0f;
  // max angular acceleration
  float max_alpha = 1.0f;
  // max curvature
  float max_curvature = 1.0f;
  // safety margin
  float safety_margin = 0.1f;

  // robot dimensions
  float  width = 0.281f;
  float  length = 0.535f;
  float  wheelbase = 0.324f;
  float  base_link_offset = 0.106f; // make this 0 for now

  // delays
  float actuation_latency = 0.2f;
  float observation_latency = 0.05f;
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
  void PopulateWallOccupancyGrid();
  std::vector<Eigen::Vector2f> GetNeighborhoodOfWallPoints(const Eigen::Vector2f p0, 
                                                           const Eigen::Vector2f p1,
                                                           float margin);
  void ObstacleDetector();

  // // Set the latency compensation object.
  void SetLatencyCompensation(LatencyCompensation* latency_compensation);

  Control GetCartesianControl(float velocity, float curvature, double time);

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
  // distance traveled
  float distance_traveled_ = 0.0f;
  // Point cloud w.r.t world frame
  std::vector<Eigen::Vector2f> point_cloud_wf;
  // Initialize variables to store cell values and resolution
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

  // robot config
  NavigationParams robot_config_;

  // Latency compensation
  LatencyCompensation* latency_compensation_;
};


}  // namespace navigation

#endif  // NAVIGATION_H
