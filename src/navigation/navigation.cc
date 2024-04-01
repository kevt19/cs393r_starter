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
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================
#include <algorithm>
#include <string>
#include <vector>
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
using std::max;
using std::min;
using std::swap;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

string GetMapFileFromName(const string& map) {
  string maps_dir_ = ros::package::getPath("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

Navigation::Navigation(const string& map_name, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0),
    ROBOT_WIDTH_(0.281),
    ROBOT_LENGTH_(0.535),
    MAX_CLEARANCE_(0.5),
    WHEEL_BASE_(0.324),
    MAX_CURVATURE_(1.0),
    MAX_ACCEL_(4.0),
    MAX_DEACCL_(9.0),
    MAX_SHORT_TERM_GOAL_(5.0),
    STOPPING_DISTANCE_(0.1),
    MAX_SPEED_(1.0),
    DELTA_T_(0.05),
    SYSTEM_LATENCY_(0.23),
    OBSTACLE_MARGIN_(0.1) {
  map_.Load(GetMapFileFromName(map_name));
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;
    return;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;

}


PathOption SelectOptimalPath(vector<PathOption> path_options, Vector2f short_term_goal){
  PathOption result;
  float w_dist_to_goal = -0.50;
  float w_clearance = 5;
  result.free_path_length = 0;
  result.clearance = 0;
  result.curvature = 0;
  float max_score = -100000.0;
  for(auto p : path_options){
    float distance = (p.closest_point - short_term_goal).norm();
    float score = p.free_path_length + w_clearance*p.clearance + w_dist_to_goal*distance;
    if (score > max_score){
      max_score = score;
      result = p;
    }
    //printf("Curvature %f, Distance to goal %f, Clearance %f, Free path length %f, score %f\n", p.curvature, distance, p.clearance, p.free_path_length, score);
  }
  return result;
}

void Navigation::ObstacleAvoidance() {
  Vector2f short_term_goal(10,0);
  // Set up initial parameters
  drive_msg_.header.stamp = ros::Time::now();
  drive_msg_.curvature = 0;
  drive_msg_.velocity = 0;

  // Time control delta
  // const float deltaT = 0.05;
  // Number of path options to take
  const float pathOptions = 20;
  vector<PathOption> path_array;

  // Iterate over all the paths
  for(float c = -MAX_CURVATURE_; c<= MAX_CURVATURE_; c+=MAX_CURVATURE_/pathOptions){
    PathOption p;
    p.curvature = c;

    //Calculate the maximum possible free path length
    if(c < kEpsilon && c > -kEpsilon){
      // Straight path
      p.free_path_length = MAX_SHORT_TERM_GOAL_;
    } else {
      float turning_radius = 1/p.curvature;
      // Using the optimization mentioned in class where we take the free path 
      // only till the tangent.
      p.free_path_length = fabs(turning_radius)*atan2(MAX_SHORT_TERM_GOAL_, fabs(turning_radius));
    }
    p = GetFreePathLength(p, short_term_goal);
    visualization::DrawPathOption(p.curvature,
                                  p.free_path_length,
                                  p.clearance,
                                  0x0000FF,
                                  false,
                                  local_viz_msg_);
      visualization::DrawCross(p.closest_point, 0.05, 0x00A000, local_viz_msg_);
      path_array.push_back(p);
    }

    PathOption p = SelectOptimalPath(path_array, short_term_goal);
    visualization::DrawPathOption(p.curvature,
                                p.free_path_length,
                                p.clearance,
                                0xFF0000,
                                true,
                                local_viz_msg_);
    visualization::DrawCross(p.closest_point, 0.1, 0xFF0000, local_viz_msg_);
    OneDTOC(p, short_term_goal);
    
    viz_pub_.publish(local_viz_msg_);
}

PathOption Navigation::GetFreePathLength(PathOption p, Eigen::Vector2f short_term_goal){
  float c = p.curvature;
  float ret_free_path_length = MAX_SHORT_TERM_GOAL_;
  float clearance = MAX_CLEARANCE_;
  Vector2f obstruction;
  if (fabs(c) < kEpsilon){
    float l = ROBOT_LENGTH_/2 + WHEEL_BASE_/2 + OBSTACLE_MARGIN_;
    float w = ROBOT_WIDTH_/2 + OBSTACLE_MARGIN_;
    for (auto &p : point_cloud_){
      if (fabs(p.y()) > w){
        continue;
      }
      float free_path_length = p.x() - l;
      if(ret_free_path_length > free_path_length){
        ret_free_path_length = free_path_length;
        obstruction = p;
      }
      ret_free_path_length = min(ret_free_path_length, p.x() - l);
    }
    for (auto &p : point_cloud_){
      if (p.x() - l > ret_free_path_length || p.x() < 0.0){
        continue;
      }
      clearance = min(clearance, fabs(p.y()));
    }
    p.free_path_length = max<float>(0.0, ret_free_path_length);
    p.clearance = clearance;
    p.obstruction = obstruction;

    Vector2f closest_point(min<float>(p.free_path_length, short_term_goal.x()), 0);
    p.closest_point = closest_point;
    return p;
  }
  float r = 1/p.curvature;
  Vector2f center(0, r);
  float r1 = fabs(r) - ROBOT_WIDTH_/2 - OBSTACLE_MARGIN_;
  float r2 = sqrt(Sq(fabs(r)+ROBOT_WIDTH_/2 + OBSTACLE_MARGIN_) + Sq(ROBOT_LENGTH_/2 + WHEEL_BASE_/2 + OBSTACLE_MARGIN_));
  vector<float> angles(point_cloud_.size());
  vector<float> distances(point_cloud_.size());
  float min_angle = M_PI;

  for(size_t i=0; i<point_cloud_.size(); i++){
    Vector2f p_i = point_cloud_[i];
    float r_obs = (p_i-center).norm();
    float a = atan2(p_i.x(), Sign(c) * (center.y() - p_i.y()));
    angles[i] = a;
    distances[i] = r_obs;
    if (a < 0.0 || r_obs < r1 || r_obs > r2){
      continue;
    }
    float free_path_length = max<float>(0, a*fabs(r) - (ROBOT_LENGTH_/2 + WHEEL_BASE_/2 + OBSTACLE_MARGIN_));
    if (free_path_length < ret_free_path_length){
      ret_free_path_length = free_path_length;
      obstruction = p_i;
      min_angle = a;
    }
  }
  for (size_t i = 0; i < point_cloud_.size(); i++){
    if (angles[i] < min_angle && angles[i] > 0.0){
      float c = fabs(distances[i] - fabs(r));
      if (clearance > c){
          clearance = c;
      }
    }
  }

  p.clearance = max<float>(0.0, clearance);
  p.free_path_length = max<float>(0.0, ret_free_path_length);
  p.obstruction = obstruction;
  // printf("Clearance %f, Free Path Length %f, Curvature %f\n", p.clearance, p.free_path_length, p.curvature);

  float closest_angle_extended = atan2(short_term_goal.x(), fabs(r- short_term_goal.y()));
  float free_angle_extended = c*p.free_path_length;
  float len_arc = fabs(closest_angle_extended)*fabs(r);
  if(len_arc < p.free_path_length){
    Vector2f closest_point(Sign(c)*r*sin(closest_angle_extended), r-r*cos(closest_angle_extended));
    p.closest_point = closest_point;
  } else{
    Vector2f closest_point(r*sin(free_angle_extended), r-r*cos(free_angle_extended));
    p.closest_point = closest_point;
  }
  return p;
}

void Navigation::OneDTOC(PathOption p, Vector2f stg){
  float dist_to_travel = p.free_path_length - 0.2;
  // float dist_to_travel = (stg - p.closest_point).norm();
  float curvature = p.curvature;
  float speed = robot_vel_.norm();
  drive_msg_.curvature = curvature;
  if (dist_to_travel < STOPPING_DISTANCE_){
    // Decelerate
    drive_msg_.velocity = max<float>(0.0, speed - DELTA_T_*MAX_DEACCL_);
  } else if (speed < MAX_SPEED_) {
    if (SYSTEM_LATENCY_ * speed > dist_to_travel) {
      drive_msg_.velocity = speed;
    } else {
      drive_msg_.velocity = min<float>(MAX_SPEED_, speed + DELTA_T_*MAX_ACCEL_);
    }
  } else{
    drive_msg_.velocity = MAX_SPEED_;
  }
  visualization::DrawPathOption(p.curvature,
                                  p.free_path_length,
                                  p.clearance,
                                  0xFF0000,
                                  true,
                                  local_viz_msg_);
  drive_pub_.publish(drive_msg_);
  viz_pub_.publish(local_viz_msg_);
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.
  PathOption p;
  p.free_path_length = 1.0;
  p.curvature = 1.0;
  p.clearance = 1.0;
  // OneDTOC(p);
  ObstacleAvoidance();
  // ObstacleAvoidance();
  // The latest observed point cloud is accessible via "point_cloud_"

  // Eventually, you will have to set the control values to issue drive commands:
  // drive_msg_.curvature = ...;
  // drive_msg_.velocity = ...;

  // Add timestamps to all messages.
  // local_viz_msg_.header.stamp = ros::Time::now();
  // global_viz_msg_.header.stamp = ros::Time::now();
  // drive_msg_.header.stamp = ros::Time::now();
  // // Publish messages.
  // viz_pub_.publish(local_viz_msg_);
  // viz_pub_.publish(global_viz_msg_);
  // drive_pub_.publish(drive_msg_);
}

}  // namespace navigation
