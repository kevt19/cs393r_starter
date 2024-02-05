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

#include <cmath>
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
#include <iostream>

// #define MAX_ACCEL 4.0
// #define MAX_SPEED 1.0
// #define CTRL_FREQ 20.0

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;

// Car Size Constants
const float CAR_LENGTH = 0.50;
const float CAR_WIDTH = 0.28;
const float SAFETY_MARGIN = 0.2;
const float BASE_TO_FRONT = 0.41;

// Kinematic Constraints
const float MAX_ACCEL = 4.0;
const float MAX_SPEED = 1.0;
const float CTRL_FREQ = 20.0;

// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;

// Scoring Weights
const float w_clearance = 0.5;
const float w_distance_to_goal = 0;
const float w_free_path_length = 0.5;

// Latency Values
const float LATENCY = 0.125;
const int MAX_CONTROLS_IN_LATENCY = static_cast<int>(ceil(LATENCY / (1 / CTRL_FREQ)));
const float TIME_PER_CONTROL = 1.0 / CTRL_FREQ;

// Path Search Parameters
const float MAX_CURVATURE = 1;
const float MIN_CURVATURE = -1;
const float NUM_POSSIBLE_PATHS = 21;

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
    nav_goal_angle_(0) {
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

void Navigation::SetNavGoal(const Eigen::Vector2f& loc, float angle) {
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

Odometry Navigation::CompensateLatencyLoc() {
  float current_x = robot_loc_.x();
  float current_y = robot_loc_.y();
  float current_omega = robot_omega_;

  double current_time = ros::Time::now().toSec();
  double min_relevant_time = current_time - LATENCY;

  // remove controls that are not relevant
  int i = 0;
  while (past_controls_.size() > 0 && past_controls_[i].time < min_relevant_time) {
    past_controls_.erase(past_controls_.begin() + i);
  }

  for(const Control& control : past_controls_) {
    float vel = control.velocity;
    float curv = control.curvature;
    // double time = control.time;

    current_x += vel * cos(current_omega) * TIME_PER_CONTROL;
    current_y += vel * sin(current_omega) * TIME_PER_CONTROL;
    current_omega += vel * curv * TIME_PER_CONTROL;
  }

  Odometry odometry;
  odometry.loc = Vector2f(current_x, current_y);
  odometry.omega = current_omega;
  return odometry;
}

vector<Vector2f> Navigation::CompensatePointCloud(const vector<Vector2f>& cloud, const Odometry& odometry) {
  vector<Vector2f> compensated_cloud;
  
  float delta_x = odometry.loc.x() - robot_loc_.x();
  float delta_y = odometry.loc.y() - robot_loc_.y();
  // float delta_omega = odometry.omega - robot_omega_;

  for (const auto& point : cloud) {
    float x = point.x() - delta_x;
    float y = point.y() - delta_y;
    compensated_cloud.push_back(Vector2f(x, y));
  }
  return compensated_cloud;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;                                     
}

double Navigation::MoveForward(double free_path_l){
  double speed = robot_vel_.norm();
  double speed_after_accel = speed + (MAX_ACCEL / CTRL_FREQ);

  // Dist that would be covered before coming to a stop if brakes are maximally applied, starting at some initial speed
  auto brake_dist = [](double initial_speed){
    return ((initial_speed * initial_speed) / (2 * MAX_ACCEL));
  };

  // Distance that would be covered by accelerating for one time step then slamming on the brakes until a stop is reached
  double accel_dist = (0.5 * (speed + speed_after_accel) * (1 / CTRL_FREQ)) + brake_dist(speed_after_accel);

  // Distance that would be covered by cruising for one time step
  double cruise_dist = (speed / CTRL_FREQ) + brake_dist(speed);
  
  if(speed < MAX_SPEED && accel_dist < free_path_l){
    // Accelerate!
    return MAX_SPEED;
  }
  else if(cruise_dist < free_path_l){
    // Cruise!
    return MAX_SPEED;
  }
  else{
    // Slam the brakes! 
    return 0.0;
  }

}

float Navigation::ComputeScore(float free_path_length, float clearance, float distance_to_goal) {
  return w_free_path_length * free_path_length + w_clearance * clearance + w_distance_to_goal * distance_to_goal;
}

float Navigation::ComputeFreePathLength(double curvature){
  double free_path_l = __DBL_MAX__;
  Vector2f target_point(0,0);

  // Special case for going forward
  if(abs(curvature) < kEpsilon){
    // Determine which points are possible obstacles
    vector<Vector2f> possible_obstacles;
    for(auto point : point_cloud_){
      if(abs(point.y()) <= ((CAR_WIDTH / 2) + SAFETY_MARGIN)){
        possible_obstacles.push_back(point);
      }
    }

    // Determine which possible obstacle is closest, and find that obstacle's free path length
    for(auto point : possible_obstacles){
      double cur_free_path_l = point.x() - (BASE_TO_FRONT + SAFETY_MARGIN);
      if (cur_free_path_l < free_path_l){
        free_path_l = cur_free_path_l;
        target_point = point;
      }
    }

    // If path is too long or there are no obstacles, go up to the goal
    if(free_path_l > nav_goal_loc_.norm() || possible_obstacles.size() <= 0){
      free_path_l = nav_goal_loc_.norm();
    }
  }

  // Otherwise, the car is turning
  else{
    double radius = 1 / curvature;
    Vector2f center_of_turn(0, radius);
    // Limit of how long the free path length can be before getting further away from the goal
    double fpl_lim = atan(nav_goal_loc_.norm() / abs(radius)) * abs(radius);
    double r1 = abs(radius) - ((CAR_WIDTH / 2) + SAFETY_MARGIN);
    double r2 = Vector2f(BASE_TO_FRONT + SAFETY_MARGIN, abs(radius) + ((CAR_WIDTH / 2) + SAFETY_MARGIN)).norm();

    // Determine which points are possible obstacles
    vector<Vector2f> possible_obstacles;
    for(auto point : point_cloud_){
      
      double theta = atan2(point.x(), (radius - point.y()));
      if((point - center_of_turn).norm() >= r1 && (point - center_of_turn).norm() <= r2 && theta > 0){
        possible_obstacles.push_back(point);
      };
    }

    // Determine which possible obstacle is closest, and find that obstacle's free path length
    for(auto point : possible_obstacles){
      double theta = atan2(point.x(), (abs(radius) - abs(point.y())));
      double omega = atan2(BASE_TO_FRONT + SAFETY_MARGIN, (abs(radius) - ((CAR_WIDTH / 2) + SAFETY_MARGIN)));
      double psi   = theta - omega;
      double cur_free_path_l = abs(radius) * psi;

      // Adjust to limit FPL to closest point of approach to the goal
      if(cur_free_path_l > fpl_lim){
        cur_free_path_l = fpl_lim;
      }

      // Choose the shortest free path length
      if (cur_free_path_l < free_path_l){
        free_path_l = cur_free_path_l;
        target_point = point;
      }
    }

    // If no obstacles, go as far as possible before getting further away from the goal
    if(possible_obstacles.size() <= 0){
      free_path_l = fpl_lim;
    }
  }

  return free_path_l;
}

float Navigation::ComputeDistanceToGoal(double curvature, double free_path_length) {
  Eigen::Vector2f endpoint;

  // Calculate where the car ends up at the end of the free path
  if(abs(curvature) < 0.05){
    endpoint = Vector2f(0, free_path_length);
  }
  else{
    double radius = 1 / curvature;
    double psi = free_path_length / abs(radius);
    Eigen::Rotation2Df rot(psi);
    endpoint = (rot * Vector2f(0, -radius)) + Vector2f(0, radius); 
  }

  return (nav_goal_loc_ - endpoint).norm();
}

float Navigation::ComputeClearance(double curvature, double free_path_length){
  return 0;
}

void Navigation::FindBestPath(double& target_curvature, double& target_free_path_l){
  double best_score = -1000;

  float curvature_range = MAX_CURVATURE - MIN_CURVATURE;
  float incr = curvature_range / (NUM_POSSIBLE_PATHS - 1);
  float curr_curv = MIN_CURVATURE;

  // Iterate over all possible paths, scoring each one
  for(int i = 0; i < NUM_POSSIBLE_PATHS; i++){
    double free_path_l = ComputeFreePathLength(curr_curv);
    if (curr_curv < 0){
      std::cout << free_path_l << std::endl;
    }
    double clearance = ComputeClearance(curr_curv, free_path_l);
    double goal_dist = ComputeDistanceToGoal(curr_curv, free_path_l);
    double curr_score = ComputeScore(free_path_l, clearance, goal_dist);

    if(curr_score > best_score){
      best_score = curr_score;
      target_curvature = curr_curv;
      target_free_path_l = free_path_l;
    }

    curr_curv += incr;
  } 
}

void Navigation::Run() {
  nav_goal_loc_ = Eigen::Vector2f(5.0, 0);
  
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.
  
  // The latest observed point cloud is accessible via "point_cloud_"


  // Eventually, you will have to set the control values to issue drive commands:
  double curvature = 0;
  double free_path_l = 0;
  FindBestPath(curvature, free_path_l);

  drive_msg_.curvature = curvature;
  drive_msg_.velocity = MoveForward(free_path_l);
  
  // add control given to Queue
  Control control;
  control.velocity = drive_msg_.velocity;
  control.curvature = drive_msg_.curvature;
  control.time = ros::Time::now().toSec();
  past_controls_.push_back(control); 

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

}  // namespace navigation
