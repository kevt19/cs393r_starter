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
const float SAFETY_MARGIN = 0.15;
const float BASE_TO_FRONT = 0.41;
const float BACK_TO_BASE = 0.135;

// Kinematic Constraints
const float MAX_ACCEL = 4.0;
const float MAX_SPEED = 1.0;
const float CTRL_FREQ = 20.0;

// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;

// Scoring Weights
const float w_clearance = 0;
const float w_distance_to_goal = 0;
const float w_free_path_length = 1;

// Latency Values
const float LATENCY = 0.125;
const int MAX_CONTROLS_IN_LATENCY = static_cast<int>(ceil(LATENCY / (1 / CTRL_FREQ)));
const float TIME_PER_CONTROL = 1.0 / CTRL_FREQ;

// Path Search Parameters
const float NUM_COARSE_GRAIN_POSSIBLE_PATHS = 31;
const float MAX_COARSE_CURVATURE = 1;
const float MIN_COARSE_CURVATURE = -1;
const float NUM_BEST_COARSE_PATHS_TO_CONSIDER = 3;
const float FINE_CURVATURE_SEARCH_RANGE = 0.1;
const float NUM_FINE_GRAIN_POSSIBLE_PATHS = 11;

// Clearance Parameters
const float MAX_CLEARANCE = 5;

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

  if(free_path_l < 0){
    return 0;
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
  double clearance = MAX_CLEARANCE;
  Eigen::Vector2f clearance_point(0,0);

  // Moving straight case
  if(abs(curvature) < kEpsilon) {
    // Determine which points will possibly affect clearance
    vector<Vector2f> clearance_points;
    for(auto point : point_cloud_){
      if(point.x() > BACK_TO_BASE && point.x() < (free_path_length + BASE_TO_FRONT + SAFETY_MARGIN - 0.1) && abs(point.y()) < MAX_CLEARANCE){
        clearance_points.push_back(point);
      }
    }

    // Determine which point is closest, and find that point's clearance
    for(auto point : clearance_points){
      double curr_clearance = abs(point.y()) - ((CAR_WIDTH / 2) + SAFETY_MARGIN);
      if (curr_clearance < clearance){
        clearance = curr_clearance;
        clearance_point = point;
      }
    }
  }

  // Moving on a curve case
  else{
    double radius = 1 / curvature;
    Eigen::Vector2f center_of_turn(0, radius);
    double r1 = abs(radius) - ((CAR_WIDTH / 2) + SAFETY_MARGIN);
    double r2 = Vector2f(BASE_TO_FRONT + SAFETY_MARGIN, abs(radius) + ((CAR_WIDTH / 2) + SAFETY_MARGIN)).norm();

    // Determine which points will possibly affect clearance
    vector<Eigen::Vector2f> clearance_points;
    for(auto point : point_cloud_){
      double theta_p = atan2(point.x(), radius - point.y());
      double omega = atan2(BASE_TO_FRONT + SAFETY_MARGIN, radius - ((CAR_WIDTH / 2) + SAFETY_MARGIN));
      double psi = free_path_length / abs(radius);
      double theta_max = omega + psi - 0.05;
      if(abs((center_of_turn - point).norm() - abs(radius)) < MAX_CLEARANCE && theta_p > 0 && theta_p < theta_max){
        clearance_points.push_back(point);
      };
    }


    // Determine which point is closest, and find that point's clearance
    for(auto point : clearance_points){
      double curr_clearance;
      if((center_of_turn - point).norm() > abs(radius)){
        curr_clearance = (center_of_turn - point).norm() - r2;
      }
      else{
        curr_clearance = r1 - (center_of_turn - point).norm();
        // Ignore points that end up with negative clearance
        if(curr_clearance < 0){
          curr_clearance = MAX_CLEARANCE;
        }
      }

      if (curr_clearance < clearance){
        clearance = curr_clearance;
        clearance_point = point;
      }
    }
  }

  // Draw point used to calculate clearance
  // visualization::DrawCross(clearance_point, 0.175, 0x00f00f, local_viz_msg_);

  return clearance;
}



void Navigation::FindBestPath(double& target_curvature, double& target_free_path_l){
  float curvature_range = MAX_COARSE_CURVATURE - MIN_COARSE_CURVATURE;
  float incr = curvature_range / (NUM_COARSE_GRAIN_POSSIBLE_PATHS - 1);
  float curr_curv = MIN_COARSE_CURVATURE;

  struct PossiblePath{
    double curv;
    double fpl;

    bool operator==(const PossiblePath& other) const {
        return false;
    }
  };
  SimpleQueue<PossiblePath, double> coarse_scores_queue;

  // Iterate over all possible paths, scoring each one
  for(int i = 0; i < NUM_COARSE_GRAIN_POSSIBLE_PATHS; i++){
    double free_path_l = ComputeFreePathLength(curr_curv);
    double clearance = ComputeClearance(curr_curv, free_path_l);
    double goal_dist = ComputeDistanceToGoal(curr_curv, free_path_l);
    double curr_score = ComputeScore(free_path_l, clearance, goal_dist);

    PossiblePath curr_path;
    curr_path.curv = curr_curv;
    curr_path.fpl = free_path_l;
    coarse_scores_queue.Push(curr_path, curr_score);

    curr_curv += incr;
  } 


  SimpleQueue<PossiblePath, double> fine_scores_queue;

  for(int i = 0; i < NUM_BEST_COARSE_PATHS_TO_CONSIDER; i++){
    PossiblePath curr_coarse_path = coarse_scores_queue.Pop();

    visualization::DrawPathOption(curr_coarse_path.curv, curr_coarse_path.fpl, 0, 0x0000ff, false, local_viz_msg_);

    incr = FINE_CURVATURE_SEARCH_RANGE / (NUM_FINE_GRAIN_POSSIBLE_PATHS - 1);
    curr_curv = curr_coarse_path.curv - (FINE_CURVATURE_SEARCH_RANGE / 2.0);

    for(int j = 0; j < NUM_FINE_GRAIN_POSSIBLE_PATHS; j++){
      double free_path_l = ComputeFreePathLength(curr_curv);
      double clearance = ComputeClearance(curr_curv, free_path_l);
      double goal_dist = ComputeDistanceToGoal(curr_curv, free_path_l);
      double curr_score = ComputeScore(free_path_l, clearance, goal_dist);

      PossiblePath curr_path;
      curr_path.curv = curr_curv;
      curr_path.fpl = free_path_l;
      fine_scores_queue.Push(curr_path, curr_score);

      visualization::DrawPathOption(curr_path.curv, curr_path.fpl, 0, 0x111111, false, local_viz_msg_);

      curr_curv += incr;
    }
  }

  PossiblePath best_path = fine_scores_queue.Pop();
  target_curvature = best_path.curv;
  target_free_path_l = best_path.fpl;

  visualization::DrawPathOption(target_curvature, target_free_path_l, 0, 0xff0000, false, local_viz_msg_);
}

void Navigation::Run() {
  nav_goal_loc_ = Eigen::Vector2f(5.0, 0);
  
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // Draw box around car
  visualization::DrawLine(Vector2f(BASE_TO_FRONT, CAR_WIDTH / 2), Vector2f(BASE_TO_FRONT, -CAR_WIDTH / 2), 0xff0000, local_viz_msg_);
  visualization::DrawLine(Vector2f(BASE_TO_FRONT, CAR_WIDTH / 2), Vector2f(-BACK_TO_BASE, CAR_WIDTH / 2), 0xff0000, local_viz_msg_);
  visualization::DrawLine(Vector2f(BASE_TO_FRONT, -CAR_WIDTH / 2), Vector2f(-BACK_TO_BASE, -CAR_WIDTH / 2), 0xff0000, local_viz_msg_);
  visualization::DrawLine(Vector2f(-BACK_TO_BASE, CAR_WIDTH / 2), Vector2f(-BACK_TO_BASE, -CAR_WIDTH / 2), 0xff0000, local_viz_msg_);

  // Draw safety margin in front of car
  visualization::DrawLine(Vector2f(BASE_TO_FRONT + SAFETY_MARGIN, CAR_WIDTH / 2 + SAFETY_MARGIN), Vector2f(BASE_TO_FRONT + SAFETY_MARGIN, -(CAR_WIDTH / 2 + SAFETY_MARGIN)), 0xff0000, local_viz_msg_);

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
