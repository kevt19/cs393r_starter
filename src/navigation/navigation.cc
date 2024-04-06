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
#include "simple_queue.h"
#include "path_options.h"
#include "latency_compensation.h"


using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
using std::max;
using std::min;

using namespace math_util;
using namespace ros_helpers;

DEFINE_double(resolution, 0.5, "Global Planner Cell resolution");

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
    latency_compensation_(new LatencyCompensation(0, 0, 0)) 
  {
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
    // run A* and generate a set of waypoints
    nav_goal_loc_ = loc;
    nav_goal_angle_ = angle; // ignore angle for now
    waypoints.clear();
    // discretize the map
    float discrete_multiplier = 1.0 / FLAGS_resolution; // makes it discrete

    // convert robot_loc and nav_goal_loc to discrete
    int robot_x_discrete = robot_loc_.x() * discrete_multiplier;
    int robot_y_discrete = robot_loc_.y() * discrete_multiplier;
    int goal_x_discrete = nav_goal_loc_.x() * discrete_multiplier;
    int goal_y_discrete = nav_goal_loc_.y() * discrete_multiplier;
    // convert back
    float robot_x = robot_x_discrete / discrete_multiplier;
    float robot_y = robot_y_discrete / discrete_multiplier;
    float goal_x = goal_x_discrete / discrete_multiplier;
    float goal_y = goal_y_discrete / discrete_multiplier;

    Eigen::Vector2f start = Eigen::Vector2f(robot_x, robot_y);
    Eigen::Vector2f goal = Eigen::Vector2f(goal_x, goal_y);

    float map_min_x = 1000.0;
    float map_min_y = 1000.0;
    float map_max_x = -1000.0;
    float map_max_y = -1000.0;

    vector<geometry::line2f> all_map_lines;
    float max_map_range = 1000.0;
    map_.GetSceneLines(robot_loc_, max_map_range, &all_map_lines);
    int all_map_lines_size = all_map_lines.size();
    for (int i = 0; i < all_map_lines_size; i++){
      Eigen::Vector2f p0 = all_map_lines[i].p0;
      Eigen::Vector2f p1 = all_map_lines[i].p1;
      map_min_x = min(map_min_x, min(p0.x(), p1.x()));
      map_min_y = min(map_min_y, min(p0.y(), p1.y()));
      map_max_x = max(map_max_x, max(p0.x(), p1.x()));
      map_max_y = max(map_max_y, max(p0.y(), p1.y()));
    }

    // let's also make dummy walls
    // try 0.5 -> 0.0, decreasing by 0.5
    float dummyValue = 0.75;
    while (dummyValue > 0.0) {
      vector<geometry::line2f> lines_list;
      // clear map
      std::map<std::pair<int, int>, bool> visited;
      std::map<std::pair<int, int>, float> distanceFromOrigin;
      std::map<std::pair<int, int>, std::pair<int, int>> waypointToParent;
      vector<Eigen::Vector2f> neighbors;
      vector<geometry::line2f> mapWallsWithDummys;
      for (int i = 0; i < all_map_lines_size; i++){
        Eigen::Vector2f p0 = all_map_lines[i].p0;
        Eigen::Vector2f p1 = all_map_lines[i].p1;
        geometry::line2f line = geometry::line2f(p0, p1);
        mapWallsWithDummys.push_back(line);

        // add dummy walls
        for (int j = -1; j <= 1; j++){
          for (int k = -1; k <= 1; k++){
            if (j == 0 && k == 0){
              continue;
            }
            Eigen::Vector2f p0_dummy = Eigen::Vector2f(p0.x() + j*dummyValue, p0.y() + k*dummyValue);
            Eigen::Vector2f p1_dummy = Eigen::Vector2f(p1.x() + j*dummyValue, p1.y() + k*dummyValue);
            geometry::line2f line_dummy = geometry::line2f(p0_dummy, p1_dummy);
            mapWallsWithDummys.push_back(line_dummy);
          }
        }
      }

      int maxIterations = 10000000;
      // Loop until we reach goal or max iterations
      std::vector<Eigen::Vector2f>::iterator neighborsIter;
      std::vector<geometry::line2f> possiblePaths;
      SimpleQueue<Eigen::Vector2f, float> searchQueue;
      // add first node to queue
      // initial distance
      float distance = (goal - start).norm();
      float x = start.x();
      float y = start.y();
      int x_discrete = x * discrete_multiplier;
      int y_discrete = y * discrete_multiplier;
      x = x_discrete / discrete_multiplier;
      y = y_discrete / discrete_multiplier;
      Eigen::Vector2f currentLocation = Eigen::Vector2f(x, y);

      searchQueue.Push(start, distance);

      int waypointIdx = 0;
      // bool finished = false;
      while (!searchQueue.Empty() && waypointIdx < maxIterations) {
        // dequeue and get x, y
        // std::pair<std::vector<Eigen::Vector2f>, Priority> currentPath = searchQueue.PopWithPriority();
        std::pair<Eigen::Vector2f, float> poppedVal = searchQueue.PopWithPriority();
        currentLocation = poppedVal.first;
        float currentCost = -1.0 * poppedVal.second;
        float estimatedDistance = (currentLocation - goal).norm();
        float actualDistanceSoFar = currentCost - estimatedDistance;

        x = currentLocation.x();
        y = currentLocation.y();
        x_discrete = x * discrete_multiplier;
        y_discrete = y * discrete_multiplier;
        x = x_discrete / discrete_multiplier;
        y = y_discrete / discrete_multiplier;

        visited[std::make_pair(x_discrete, y_discrete)] = true;

        // check if we already reached the goal
        // Eigen::Vector2f wp = Eigen::Vector2f(x, y);
        if ((currentLocation - goal).norm() < FLAGS_resolution){
          printf("Current location x: %f, y: %f\n", x, y);
          printf("Reached goal\n");

          // edge case: robot is already at goal, then just return
          if (waypointIdx == 0){
            break;
          }

          // copy over waypoints in reverse
          vector<Eigen::Vector2f> reversedWaypoints;
          Eigen::Vector2f backtrackLocation = currentLocation;
          std::pair<int, int> backtrack = std::make_pair(x_discrete, y_discrete);
          bool has_parent = true;
          while (has_parent){
            printf("current x: %d, y: %d\n", backtrack.first, backtrack.second);
            x_discrete = backtrack.first;
            y_discrete = backtrack.second;
            x = x_discrete / discrete_multiplier;
            y = y_discrete / discrete_multiplier;

            backtrackLocation = Eigen::Vector2f(x, y);

            reversedWaypoints.push_back(backtrackLocation);

            // try to find parent
            if (waypointToParent.find(std::make_pair(x_discrete, y_discrete)) == waypointToParent.end()){
              has_parent = false;
              break;
            }
            backtrack = waypointToParent[std::make_pair(x_discrete, y_discrete)];
          }
          // copy over waypoints in reverse
          std::vector<Eigen::Vector2f>::iterator waypointsIter;
          for (waypointsIter = reversedWaypoints.end() - 1; waypointsIter != reversedWaypoints.begin() - 1; waypointsIter--){
            waypoints.push_back(*waypointsIter);
          }
          break;
        }
        neighbors.clear();
        possiblePaths.clear();

        // check if current x, y is in the map
        for (int i = -1; i <= 1; i++){
          for (int j = -1; j <= 1; j++){
            if (i == 0 && j == 0){
              continue;
            }
            if (x + i*FLAGS_resolution < map_min_x || x + i*FLAGS_resolution > map_max_x || y + j*FLAGS_resolution < map_min_y || y + j*FLAGS_resolution > map_max_y){
              continue;
            }
            neighbors.push_back(Eigen::Vector2f(x + i*FLAGS_resolution, y + j*FLAGS_resolution));
          }
        } 
        float max_range = 3.0 * FLAGS_resolution / 2;

        const float x_min = x - max_range;
        const float y_min = y - max_range;
        const float x_max = x + max_range;
        const float y_max = y + max_range;
        lines_list.clear();
        for (const geometry::line2f& l : mapWallsWithDummys) {
          if (l.p0.x() < x_min && l.p1.x() < x_min) continue;
          if (l.p0.y() < y_min && l.p1.y() < y_min) continue;
          if (l.p0.x() > x_max && l.p1.x() > x_max) continue;
          if (l.p0.y() > y_max && l.p1.y() > y_max) continue;
          lines_list.push_back(l);
        }

        // Eigen::Vector2f currentLocation = Eigen::Vector2f(x, y);
        // map_.GetSceneLines(currentLocation, max_range, &lines_list);
        // create line segment from x, y to neighbor
        for (neighborsIter = neighbors.begin(); neighborsIter != neighbors.end(); neighborsIter++){
          Eigen::Vector2f neighbor = *neighborsIter;  
          float neighbor_x = neighbor.x();
          float neighbor_y = neighbor.y();

          int x_lookup = neighbor_x * discrete_multiplier;
          int y_lookup = neighbor_y * discrete_multiplier;
          if (visited[std::make_pair(x_lookup, y_lookup)]) {
            continue;
          }

          bool validNeighbor = true;
          int n_lines = lines_list.size();
          geometry::line2f currentLine = geometry::line2f(x, y, neighbor_x, neighbor_y);
          for (int j = 0; j < n_lines; j++){
            // printf("Checking intersection with map line x1 %f, y1 %f, x2 %f, y2 %f\n", lines_list[j].p0.x(), lines_list[j].p0.y(), lines_list[j].p1.x(), lines_list[j].p1.y());
            if (lines_list[j].Intersects(currentLine)){
              // if it intersects, remove the line segment
              validNeighbor = false;
              break;
            }
          }
          if (validNeighbor){
            possiblePaths.push_back(currentLine);
          }
        }
        
        int nPossiblePaths = possiblePaths.size();
        for (int i = 0; i < nPossiblePaths; i++){
          Eigen::Vector2f selectedWaypoint = possiblePaths[i].p1;

          float next_x = selectedWaypoint.x();
          float next_y = selectedWaypoint.y();
          int next_x_discrete = next_x * discrete_multiplier;
          int next_y_discrete = next_y * discrete_multiplier;
          next_x = next_x_discrete / discrete_multiplier;
          next_y = next_y_discrete / discrete_multiplier;

          // let's use actual distance for distance calculation
          float distanceToNode = (selectedWaypoint - currentLocation).norm(); 
          // float distanceToNode = FLAGS_resolution;
          float h = (selectedWaypoint - goal).norm();
          float g = actualDistanceSoFar + distanceToNode;

          // if it does not have a parent, add it to the parent map
          if (waypointToParent.find(std::make_pair(next_x_discrete, next_y_discrete)) == waypointToParent.end()){
            waypointToParent[std::make_pair(next_x_discrete, next_y_discrete)] = std::make_pair(x_discrete, y_discrete);
            distanceFromOrigin[std::make_pair(next_x_discrete, next_y_discrete)] = g;
          }
          else
          {
            // check if distance is less than current distance
            float distanceFromExistingParent = distanceFromOrigin[std::make_pair(next_x_discrete, next_y_discrete)];
            if (distanceFromExistingParent > g){
              distanceFromOrigin[std::make_pair(next_x_discrete, next_y_discrete)] = g;
              waypointToParent[std::make_pair(next_x_discrete, next_y_discrete)] = std::make_pair(x_discrete, y_discrete);
            }
            else{
              g = distanceFromExistingParent; // otherwise let's use it
            }
          }
          searchQueue.Push(selectedWaypoint, -1.0 * (g + h));
        }
        waypointIdx++;
      }
      if (waypoints.size() == 0) {
        printf("No waypoints found, will try lower dummy value if > 0.0\n");
        dummyValue -= 0.05;
      }
      else {
        nav_goal_loc_ = waypoints[0];
        printf("Waypoints found\n");
        break;
      }
    }
}

void Navigation::ObstacleDetector() {
  // Create a pointcloud that is w.r.t the map frame
  for (const auto& point: point_cloud_) {
    Eigen::Rotation2Df r_map_to_baselink(robot_angle_);
    Eigen::Matrix2f R = r_map_to_baselink.toRotationMatrix();
    Eigen::Matrix3f T_map_to_baselink;
    T_map_to_baselink << R(0,0), R(0,1), robot_loc_.x(),
                        R(1,0), R(1,1), robot_loc_.y(),
                        0,      0,      1;

    Eigen::Vector3f point_to_transform(point.x(), point.y(), 1.0f);
    Eigen::Vector3f transformed_point = T_map_to_baselink * point_to_transform;
    Eigen::Vector2f point_wf(transformed_point[0], transformed_point[1]);
    point_cloud_wf.push_back(point_wf);
  }

    // // Create an obstacle based on how many points fall within the cell
      int point_count_threshold = 10;
      float discrete_multiplier = 1.0 / FLAGS_resolution; // makes it discrete

      for (Vector2f point: point_cloud_wf) {
        int cellX = std::round(point.x() * discrete_multiplier);
        int cellY = std::round(point.y() * discrete_multiplier);
        std::pair<int, int> cell = std::make_pair(cellX, cellY);
      
        // Increment count for the cell
        obstacle_map[cell]++;

        // Check if the count exceeds the threshold
        if (obstacle_map[cell] > point_count_threshold) {
            // std::cout << "Cell (" << cellX << ", " << cellY << ") has more than " << point_count_threshold << " points." << std::endl;
            detected_obstacles[cell] = true; 
        }
    }

    for (const auto& cell: detected_obstacles) {
            visualization::DrawCross(Vector2f(cell.first.first/2, cell.first.second/2), 0.05, 0x000000, global_viz_msg_);
            printf("Cell: (%d, %d)\n", cell.first.first, cell.first.second);
          }

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
  latency_compensation_->recordObservation(loc[0], loc[1], angle, ros::Time::now().toSec());
  Observation predictedState = latency_compensation_->getPredictedState();
  odom_loc_ = {predictedState.x, predictedState.y};
  odom_angle_ = predictedState.theta;
  robot_vel_ = {predictedState.vx, predictedState.vy};
  robot_omega_ = predictedState.omega;

  point_cloud_ = latency_compensation_->forward_predict_point_cloud(point_cloud_, predictedState.x, predictedState.y, predictedState.theta);
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;                            
}

void Navigation::SetLatencyCompensation(LatencyCompensation* latency_compensation) {
  latency_compensation_ = latency_compensation;
}

// Convert (velocity, curvature) to (x_dot, y_dot, theta_dot)
Control Navigation::GetCartesianControl(float velocity, float curvature, double time) {
  float x_dot = velocity * cos(curvature);
  float y_dot = velocity * sin(curvature);
  float theta_dot = velocity * curvature;

  return {x_dot, y_dot, theta_dot, time};
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
  
  // The latest observed point cloud is accessible via "point_cloud_"

  // Eventually, you will have to set the control values to issue drive commands:
  // drive_msg_.curvature = ...;
  // drive_msg_.velocity = ...;
  float current_speed = robot_vel_.norm();
  // cout << current_speed << endl;
  // distance_traveled_ += current_speed * robot_config_.dt;
  // float dist_to_go = (10 - distance_traveled_); // hard code to make it go 10 forward
  // float cmd_vel = run1DTimeOptimalControl(dist_to_go, current_speed, robot_config_);

  vector<PathOption> path_options = samplePathOptions(31, point_cloud_, robot_config_);
  int best_path = selectPath(path_options, nav_goal_loc_, robot_angle_, robot_loc_);

  drive_msg_.curvature = path_options[best_path].curvature;
  drive_msg_.velocity = run1DTimeOptimalControl(path_options[best_path].free_path_length, current_speed, robot_config_);
	
  // cout << drive_msg_.curvature << " " << drive_msg_.velocity << endl;

  // visualization here
  // visualization::DrawRectangle(Vector2f(robot_config_.length/2 - robot_config_.base_link_offset, 0),
  //     robot_config_.length, robot_config_.width, 0, 0x00FF00, local_viz_msg_);
  // Draw all path options in blue
  for (unsigned int i = 0; i < path_options.size(); i++) {
      visualization::DrawPathOption(path_options[i].curvature, path_options[i].free_path_length, 0, 0x0000FF, false, local_viz_msg_);
  }
  // Draw the best path in red
  visualization::DrawPathOption(path_options[best_path].curvature, path_options[best_path].free_path_length, path_options[best_path].clearance, 0xFF0000, true, local_viz_msg_);
// Find the closest point in the point cloud

  // Plot the closest point in purple
  visualization::DrawLine(path_options[best_path].closest_point, Vector2f(0, 1/path_options[best_path].curvature), 0xFF00FF, local_viz_msg_);
  // for debugging
  
    
  visualization::DrawPoint(Vector2f(0, 1/path_options[best_path].curvature), 0x0000FF, local_viz_msg_);

// Determine if we already reached a waypoint
  if (waypoints.size() > 0){
    for (unsigned int i = 0; i < waypoints.size(); i++){
      Eigen::Vector2f waypoint = waypoints[i];
      float distance = (waypoint - robot_loc_).norm();
      if (distance < 1){
        // then we start from the next waypoint and erase the others
        waypoints.erase(waypoints.begin(), waypoints.begin() + i + 1);
        if (waypoints.size() > 0) {
          nav_goal_loc_ = waypoints[0];
        }
        else {
          nav_goal_loc_ = robot_loc_;
        }
        break;
      }
    }
  }

  // Draw waypoints from A*
  // iterate over waypoints
  vector<Eigen::Vector2f>::iterator iter;
  Eigen::Vector2f previousWaypoint = Eigen::Vector2f(0, 0);
  for (iter = waypoints.begin(); iter != waypoints.end(); iter++){
    Eigen::Vector2f waypoint = *iter;
    // swap out x, y
    float x = waypoint.x();
    float y = waypoint.y();
    // convert from global frame to robot frame
    float rotation = robot_angle_;
    float translationX = robot_loc_.x();
    float translationY = robot_loc_.y();
    float newX = cos(rotation) * (x - translationX) + sin(rotation) * (y - translationY);
    float newY = -sin(rotation) * (x - translationX) + cos(rotation) * (y - translationY);
    Eigen::Vector2f robot_frame_waypoint = Eigen::Vector2f(newX, newY);
    visualization::DrawCross(robot_frame_waypoint, 0.15, 0x000000, local_viz_msg_);
    // draw line to waypoint
    visualization::DrawLine(previousWaypoint, robot_frame_waypoint, 0x000000, local_viz_msg_);
    previousWaypoint = robot_frame_waypoint;
  }

  ObstacleDetector();


  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
  // Record control for latency compensation
  Control control = GetCartesianControl(drive_msg_.velocity, drive_msg_.curvature, drive_msg_.header.stamp.toSec());
  latency_compensation_->recordControl(control);

  // Hack because ssh -X is slow
  // if (latency_compensation_->getControlQueue().size() == 100) {
  //  exit(0);
}

} // namespace navigation
