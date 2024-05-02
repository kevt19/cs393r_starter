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
#include "simple_queue.h"
#include "navigation.h"
#include "visualization/visualization.h"
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using Eigen::Vector2f;
using std::max;
using std::min;
using std::string;
using std::swap;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

DEFINE_double(resolution, 0.25, "Global Planner Cell resolution");
DEFINE_double(max_wall_margin, 0.75, "Maximum margin to consider for walls");

DEFINE_bool(usingSlamMap, true, "Use the slam map for navigation");

namespace
{
  ros::Publisher drive_pub_;
  ros::Publisher viz_pub_;
  VisualizationMsg local_viz_msg_;
  VisualizationMsg global_viz_msg_;
  AckermannCurvatureDriveMsg drive_msg_;
  // Epsilon value for handling limited numerical precision.
  const float kEpsilon = 1e-5;
} // namespace

namespace navigation
{

  string GetMapFileFromName(const string &map)
  {
    string maps_dir_ = ros::package::getPath("amrl_maps");
    return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
  }

  Navigation::Navigation(const string &map_name, ros::NodeHandle *n) : odom_initialized_(false),
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
                                                                       OBSTACLE_MARGIN_(0.1)
  {
    if (!FLAGS_usingSlamMap)
    {
      map_.Load(GetMapFileFromName(map_name));
    }
    last_map_update_time_ = GetWallTime();
    drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
        "ackermann_curvature_drive", 1);
    viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
    local_viz_msg_ = visualization::NewVisualizationMessage(
        "base_link", "navigation_local");
    global_viz_msg_ = visualization::NewVisualizationMessage(
        "map", "navigation_global");
    InitRosHeader("base_link", &drive_msg_.header);
  }

  void Navigation::ReloadMap(std::string map_name)
  {
    map_.Load(map_name); // also need to replan
    if (waypoints.size() > 0)
    {
      nav_goal_loc_ = waypoints[waypoints.size() - 1];
      SetNavGoal(nav_goal_loc_, nav_goal_angle_);
    }
  }

  std::vector<Eigen::Vector2f> Navigation::GetNeighborhoodOfWallPoints(
      const Eigen::Vector2f p0,
      const Eigen::Vector2f p1,
      float margin)
  {

    // Delta between points
    Eigen::Vector2f delta_p = p1 - p0;

    // Calculate angle and rotation matrix
    double angle = atan2(delta_p[1], delta_p[0]);

    Eigen::Rotation2Df transform_wrt_line(angle);
    Eigen::Matrix2f R = transform_wrt_line.toRotationMatrix();
    Eigen::Matrix3f T_wrt_line;
    T_wrt_line << R(0, 0), R(0, 1), p0.x(),
        R(1, 0), R(1, 1), p0.y(),
        0, 0, 1;

    // Calculate line length
    double lineLength = sqrt(pow(delta_p[0], 2) + pow(delta_p[1], 2));

    // Generate margins and distances
    std::vector<double> margins;
    for (double m = -1.0 * margin; m < margin; m += 0.05)
    {
      margins.push_back(m);
    }
    if (margin < FLAGS_resolution)
      margins.push_back(margin);

    std::vector<double> distances;
    for (double d = 0; d < lineLength; d += 0.05)
    {
      distances.push_back(d);
    }

    // Generate points
    std::vector<Eigen::Vector2f> points;
    for (double m : margins)
    {
      for (double d : distances)
      {
        Eigen::Vector3f point_to_transform(d, m, 1.0f);
        Eigen::Vector3f homogeneous_transformed_point = T_wrt_line * point_to_transform;
        Eigen::Vector2f transformed_point(homogeneous_transformed_point[0], homogeneous_transformed_point[1]);
        points.push_back(transformed_point);
      }
    }
    return points;
  }

  void Navigation::PopulateWallOccupancyGrid()
  {
    double discrete_multiplier = 1.0 / FLAGS_resolution; // makes it discrete
    vector<geometry::line2f> all_map_lines;
    float max_map_range = 1000.0;
    map_.GetSceneLines(robot_loc_, max_map_range, &all_map_lines);
    int all_map_lines_size = all_map_lines.size();

    for (float marginParallel = FLAGS_max_wall_margin; marginParallel > 0; marginParallel -= 0.05)
    {
      // printf("Margin: %f\n", marginParallel);
      std::map<std::pair<int, int>, bool> wallOccupancyGrid;
      for (int i = 0; i < all_map_lines_size; i++)
      {
        Eigen::Vector2f p0 = all_map_lines[i].p0;
        Eigen::Vector2f p1 = all_map_lines[i].p1;

        // Get the points that are parallel to the wall
        std::vector<Eigen::Vector2f> wallPoints = GetNeighborhoodOfWallPoints(p0, p1, marginParallel);
        for (Eigen::Vector2f point : wallPoints)
        {
          int cellX = std::round(point.x() * discrete_multiplier);
          int cellY = std::round(point.y() * discrete_multiplier);
          std::pair<int, int> cell = std::make_pair(cellX, cellY);
          wallOccupancyGrid[cell] = true;
        }
      }
      wall_occupancy_grids.push_back(wallOccupancyGrid);
    }
    // add empty wall occupancy grid to make code more straightforward
    std::map<std::pair<int, int>, bool> emptyWallOccupancyGrid;
    wall_occupancy_grids.push_back(emptyWallOccupancyGrid);
  }

  void Navigation::SetNavGoal(const Vector2f &loc, float angle)
  {
    // run A* and generate a set of waypoints
    nav_goal_loc_ = loc;
    nav_goal_angle_ = angle; // ignore angle for now
    waypoints.clear();
    global_plan_grid.clear();
    // discretize the map
    float discrete_multiplier = 1.0 / FLAGS_resolution; // makes it discrete

    // convert robot_loc and nav_goal_loc to discrete
    int goal_x_discrete = nav_goal_loc_.x() * discrete_multiplier;
    int goal_y_discrete = nav_goal_loc_.y() * discrete_multiplier;

    // use actual values for start, otherwise possibility of going through walls!
    // (because of rounding)
    float robot_x = robot_loc_.x();
    float robot_y = robot_loc_.y();
    float goal_x = goal_x_discrete / discrete_multiplier;
    float goal_y = goal_y_discrete / discrete_multiplier;

    Eigen::Vector2f start = Eigen::Vector2f(robot_x, robot_y);
    Eigen::Vector2f goal = Eigen::Vector2f(goal_x, goal_y);

    float map_min_x = 10000.0;
    float map_min_y = 10000.0;
    float map_max_x = -10000.0;
    float map_max_y = -10000.0;

    vector<geometry::line2f> all_map_lines;
    float max_map_range = 1000.0;
    map_.GetSceneLines(robot_loc_, max_map_range, &all_map_lines);
    int all_map_lines_size = all_map_lines.size();
    for (int i = 0; i < all_map_lines_size; i++)
    {
      Eigen::Vector2f p0 = all_map_lines[i].p0;
      Eigen::Vector2f p1 = all_map_lines[i].p1;
      map_min_x = min(map_min_x, min(p0.x(), p1.x()));
      map_min_y = min(map_min_y, min(p0.y(), p1.y()));
      map_max_x = max(map_max_x, max(p0.x(), p1.x()));
      map_max_y = max(map_max_y, max(p0.y(), p1.y()));
    }

    int n_wall_occupancy_grids = wall_occupancy_grids.size();
    printf("Number of wall occupancy grids: %d\n", n_wall_occupancy_grids);
    for (int i = 0; i < n_wall_occupancy_grids; i++)
    {
      std::map<std::pair<int, int>, bool> wallOccupancyGrid = wall_occupancy_grids[i];
      // A* search

      std::map<std::pair<int, int>, bool> visited;
      std::map<std::pair<int, int>, float> distanceFromOrigin;
      std::map<std::pair<int, int>, std::pair<int, int>> waypointToParent;
      vector<Eigen::Vector2f> neighbors;

      int maxIterations = 10000000; // for A*
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
      while (!searchQueue.Empty() && waypointIdx < maxIterations)
      {
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
        if ((currentLocation - goal).norm() < FLAGS_resolution)
        {
          printf("Current location x: %f, y: %f\n", x, y);
          printf("Reached goal\n");

          // edge case: robot is already at goal, then just return
          if (waypointIdx == 0)
          {
            break;
          }

          // copy over waypoints in reverse
          vector<Eigen::Vector2f> reversedWaypoints;
          Eigen::Vector2f backtrackLocation = currentLocation;
          std::pair<int, int> backtrack = std::make_pair(x_discrete, y_discrete);
          bool has_parent = true;
          while (has_parent)
          {
            x_discrete = backtrack.first;
            y_discrete = backtrack.second;
            x = x_discrete / discrete_multiplier;
            y = y_discrete / discrete_multiplier;

            backtrackLocation = Eigen::Vector2f(x, y);

            reversedWaypoints.push_back(backtrackLocation);
            std::pair<int, int> lookup = std::make_pair(x_discrete, y_discrete);
            // add to global plan
            global_plan_grid[lookup] = true;

            // try to find parent
            if (waypointToParent.find(lookup) == waypointToParent.end())
            {
              has_parent = false;
              break;
            }
            backtrack = waypointToParent[lookup];
          }
          // copy over waypoints in reverse
          std::vector<Eigen::Vector2f>::iterator waypointsIter;
          for (waypointsIter = reversedWaypoints.end() - 1; waypointsIter != reversedWaypoints.begin() - 1; waypointsIter--)
          {
            waypoints.push_back(*waypointsIter);
          }
          break;
        }
        neighbors.clear();
        possiblePaths.clear();

        // check if current x, y is in the map
        for (int i = -1; i <= 1; i++)
        {
          for (int j = -1; j <= 1; j++)
          {
            if (i == 0 && j == 0)
            {
              continue;
            }
            if (x + i * FLAGS_resolution < map_min_x || x + i * FLAGS_resolution > map_max_x || y + j * FLAGS_resolution < map_min_y || y + j * FLAGS_resolution > map_max_y)
            {
              continue;
            }
            neighbors.push_back(Eigen::Vector2f(x + i * FLAGS_resolution, y + j * FLAGS_resolution));
          }
        }
        float max_range = 3.0 * FLAGS_resolution / 2;

        vector<geometry::line2f> neabyWalls;

        Eigen::Vector2f currentLocation = Eigen::Vector2f(x, y);
        map_.GetSceneLines(currentLocation, max_range, &neabyWalls);
        // create line segment from x, y to neighbor
        for (neighborsIter = neighbors.begin(); neighborsIter != neighbors.end(); neighborsIter++)
        {
          Eigen::Vector2f neighbor = *neighborsIter;
          float neighbor_x = neighbor.x();
          float neighbor_y = neighbor.y();

          int x_lookup = neighbor_x * discrete_multiplier;
          int y_lookup = neighbor_y * discrete_multiplier;
          std::pair<int, int> lookup = std::make_pair(x_lookup, y_lookup);
          if (visited[lookup])
          {
            continue;
          }

          // check if key exists in obstacle occupancy grid
          if (obstacle_occupancy_grid.find(lookup) != obstacle_occupancy_grid.end())
          {
            continue;
          }

          // check if key exists in wall occupancy grid
          if (wallOccupancyGrid.find(lookup) != wallOccupancyGrid.end())
          {
            continue;
          }

          // final check if it intersects with any walls
          geometry::line2f currentLine = geometry::line2f(x, y, neighbor_x, neighbor_y);
          possiblePaths.push_back(currentLine);
          bool validNeighbor = true;
          int n_lines = neabyWalls.size();
          for (int j = 0; j < n_lines; j++)
          {
            if (neabyWalls[j].Intersects(currentLine))
            {
              validNeighbor = false;
              break;
            }
          }
          if (validNeighbor)
          {
            possiblePaths.push_back(currentLine);
          }
        }

        int nPossiblePaths = possiblePaths.size();
        for (int i = 0; i < nPossiblePaths; i++)
        {
          Eigen::Vector2f selectedWaypoint = possiblePaths[i].p1;
          float next_x = selectedWaypoint.x();
          float next_y = selectedWaypoint.y();
          int next_x_discrete = next_x * discrete_multiplier;
          int next_y_discrete = next_y * discrete_multiplier;
          next_x = next_x_discrete / discrete_multiplier;
          next_y = next_y_discrete / discrete_multiplier;

          float distanceToNode = (selectedWaypoint - currentLocation).norm();
          float h = (selectedWaypoint - goal).norm();
          float g = actualDistanceSoFar + distanceToNode;

          // if it does not have a parent, add it to the parent map
          if (waypointToParent.find(std::make_pair(next_x_discrete, next_y_discrete)) == waypointToParent.end())
          {
            waypointToParent[std::make_pair(next_x_discrete, next_y_discrete)] = std::make_pair(x_discrete, y_discrete);
            distanceFromOrigin[std::make_pair(next_x_discrete, next_y_discrete)] = g;
          }
          else
          {
            // check if distance is less than current distance
            float distanceFromExistingParent = distanceFromOrigin[std::make_pair(next_x_discrete, next_y_discrete)];
            if (distanceFromExistingParent > g)
            {
              distanceFromOrigin[std::make_pair(next_x_discrete, next_y_discrete)] = g;
              waypointToParent[std::make_pair(next_x_discrete, next_y_discrete)] = std::make_pair(x_discrete, y_discrete);
            }
            else
            {
              g = distanceFromExistingParent; // otherwise let's use it
            }
          }
          searchQueue.Push(selectedWaypoint, -1.0 * (g + h)); // negative because we want to pop the smallest value
        }
        waypointIdx++;
      }
      if (waypoints.size() == 0)
      {
        printf("No waypoints found, will try less strict wall occupancy grid...\n");
      }
      else
      {
        nav_goal_loc_ = waypoints[0];
        printf("Waypoints found\n");
        break;
      }
    }
  }

  void Navigation::UpdateLocation(const Eigen::Vector2f &loc, float angle)
  {
    localization_initialized_ = true;
    robot_loc_ = loc;
    robot_angle_ = angle;
  }

  void Navigation::UpdateOdometry(const Vector2f &loc,
                                  float angle,
                                  const Vector2f &vel,
                                  float ang_vel)
  {
    robot_omega_ = ang_vel;
    robot_vel_ = vel;
    if (!odom_initialized_)
    {
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

  void Navigation::ObservePointCloud(const vector<Vector2f> &cloud,
                                     double time)
  {
    point_cloud_ = cloud;
  }

  PathOption SelectOptimalPath(vector<PathOption> path_options, Vector2f short_term_goal)
  {
    PathOption result;
    float w_dist_to_goal = -2.0;
    float w_clearance = 5.0;
    float w_free_path = 0.5;
    result.free_path_length = 0;
    result.clearance = 0;
    result.curvature = 0;
    float max_score = -10000.0;
    for (auto p : path_options)
    {
      // if (fabs(p.curvature) < kEpsilon)
      // {
      //   continue;
      // }
      float distance = (p.closest_point - short_term_goal).norm();
      float score = w_free_path * p.free_path_length + w_clearance * p.clearance + w_dist_to_goal * distance;
      if (score > max_score)
      {
        max_score = score;
        result = p;
      }
      // printf("Curvature %f, Distance to goal %f, Clearance %f, Free path length %f, score %f\n", p.curvature, distance, p.clearance, p.free_path_length, score);
    }
    return result;
  }

  void Navigation::LocalObstacleAvoidance()
  {
    Eigen::Rotation2Df r_map_to_baselink(-1.0 * robot_angle_);
    Eigen::Matrix2f R = r_map_to_baselink.toRotationMatrix();
    Eigen::Matrix3f T_map_to_baselink;

    float translationX = robot_loc_.x();
    float translationY = robot_loc_.y();

    T_map_to_baselink << R(0, 0), R(0, 1), 0,
        R(1, 0), R(1, 1), 0,
        0, 0, 1;

    Eigen::Vector3f translation(translationX, translationY, 0.0f);

    float goal_x = nav_goal_loc_.x();
    float goal_y = nav_goal_loc_.y();
    Eigen::Vector3f point_to_transform(goal_x, goal_y, 1.0f);

    point_to_transform = point_to_transform - translation;
    Eigen::Vector3f transformed_point = T_map_to_baselink * point_to_transform;
    Eigen::Vector2f short_term_goal(transformed_point[0], transformed_point[1]);

    // let's correct it to be in robots frame of view
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
    for (float c = -MAX_CURVATURE_; c <= MAX_CURVATURE_; c += MAX_CURVATURE_ / pathOptions)
    {
      PathOption p;
      p.curvature = c;

      // Calculate the maximum possible free path length
      if (c < kEpsilon && c > -kEpsilon)
      {
        // Straight path
        p.free_path_length = MAX_SHORT_TERM_GOAL_;
      }
      else
      {
        float turning_radius = 1 / p.curvature;
        // Using the optimization mentioned in class where we take the free path
        // only till the tangent.
        p.free_path_length = fabs(turning_radius) * atan2(MAX_SHORT_TERM_GOAL_, fabs(turning_radius));
      }
      p = GetFreePathLength(p, short_term_goal);

      // Limit the free path length to the closest point of approach to the nav goal
      float turning_radius = 1 / p.curvature;
      float fpl_lim_curve = fabs(turning_radius) * atan2(short_term_goal.norm(), fabs(turning_radius));
      if (c < kEpsilon && c > -kEpsilon && p.free_path_length > short_term_goal.norm())
      {
        // Straight path
        p.free_path_length = short_term_goal.norm();
      }
      else if (p.free_path_length > fpl_lim_curve)
      {
        // Using the optimization mentioned in class where we take the free path
        // only till the tangent.
        p.free_path_length = fpl_lim_curve;
      }

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
  }

  PathOption Navigation::GetFreePathLength(PathOption p, Eigen::Vector2f short_term_goal)
  {
    float c = p.curvature;
    float ret_free_path_length = MAX_SHORT_TERM_GOAL_;
    float clearance = MAX_CLEARANCE_;
    Vector2f obstruction;
    if (fabs(c) < kEpsilon)
    {
      float l = ROBOT_LENGTH_ / 2 + WHEEL_BASE_ / 2 + OBSTACLE_MARGIN_;
      float w = ROBOT_WIDTH_ / 2 + OBSTACLE_MARGIN_;
      for (auto &p : point_cloud_)
      {
        if (fabs(p.y()) > w)
        {
          continue;
        }
        float free_path_length = p.x() - l;
        if (ret_free_path_length > free_path_length)
        {
          ret_free_path_length = free_path_length;
          obstruction = p;
        }
        ret_free_path_length = min(ret_free_path_length, p.x() - l);
      }
      for (auto &p : point_cloud_)
      {
        if (p.x() - l > ret_free_path_length || p.x() < 0.0)
        {
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
    float r = 1 / p.curvature;
    Vector2f center(0, r);
    float r1 = fabs(r) - ROBOT_WIDTH_ / 2 - OBSTACLE_MARGIN_;
    float r2 = sqrt(Sq(fabs(r) + ROBOT_WIDTH_ / 2 + OBSTACLE_MARGIN_) + Sq(ROBOT_LENGTH_ / 2 + WHEEL_BASE_ / 2 + OBSTACLE_MARGIN_));
    vector<float> angles(point_cloud_.size());
    vector<float> distances(point_cloud_.size());
    float min_angle = M_PI;

    for (size_t i = 0; i < point_cloud_.size(); i++)
    {
      Vector2f p_i = point_cloud_[i];
      float r_obs = (p_i - center).norm();
      float a = atan2(p_i.x(), Sign(c) * (center.y() - p_i.y()));
      angles[i] = a;
      distances[i] = r_obs;
      if (a < 0.0 || r_obs < r1 || r_obs > r2)
      {
        continue;
      }
      float free_path_length = max<float>(0, a * fabs(r) - (ROBOT_LENGTH_ / 2 + WHEEL_BASE_ / 2 + OBSTACLE_MARGIN_));
      if (free_path_length < ret_free_path_length)
      {
        ret_free_path_length = free_path_length;
        obstruction = p_i;
        min_angle = a;
      }
    }
    for (size_t i = 0; i < point_cloud_.size(); i++)
    {
      if (angles[i] < min_angle && angles[i] > 0.0)
      {
        float c = fabs(distances[i] - fabs(r));
        if (clearance > c)
        {
          clearance = c;
        }
      }
    }

    p.clearance = max<float>(0.0, clearance);
    p.free_path_length = max<float>(0.0, ret_free_path_length);

    p.obstruction = obstruction;
    // printf("Clearance %f, Free Path Length %f, Curvature %f\n", p.clearance, p.free_path_length, p.curvature);

    float closest_angle_extended = atan2(short_term_goal.x(), fabs(r - short_term_goal.y()));
    float free_angle_extended = c * p.free_path_length;
    float len_arc = fabs(closest_angle_extended) * fabs(r);
    if (len_arc < p.free_path_length)
    {
      Vector2f closest_point(Sign(c) * r * sin(closest_angle_extended), r - r * cos(closest_angle_extended));
      p.closest_point = closest_point;
    }
    else
    {
      Vector2f closest_point(r * sin(free_angle_extended), r - r * cos(free_angle_extended));
      p.closest_point = closest_point;
    }
    return p;
  }

  void Navigation::OneDTOC(PathOption p, Vector2f stg)
  {
    float dist_to_travel = p.free_path_length - 0.2;
    // float dist_to_travel = (stg - p.closest_point).norm();
    float curvature = p.curvature;
    float speed = robot_vel_.norm();
    drive_msg_.curvature = curvature;
    if (dist_to_travel < STOPPING_DISTANCE_)
    {
      // Decelerate
      drive_msg_.velocity = max<float>(0.0, speed - DELTA_T_ * MAX_DEACCL_);
    }
    else if (speed < MAX_SPEED_)
    {
      if (SYSTEM_LATENCY_ * speed > dist_to_travel)
      {
        drive_msg_.velocity = speed;
      }
      else
      {
        drive_msg_.velocity = min<float>(MAX_SPEED_, speed + DELTA_T_ * MAX_ACCEL_);
      }
    }
    else
    {
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

  Eigen::Vector2f Navigation::ClosestPointOnLine(const geometry::line2f line_seg, const Eigen::Vector2f point)
  {
    Vector2f line_vec = line_seg.p1 - line_seg.p0;
    Vector2f point_to_line_vec = point - line_seg.p0;
    double projection = point_to_line_vec.dot(line_vec) / line_vec.dot(line_vec);
    projection = std::max(0.0, std::min(1.0, projection));
    Vector2f closest_point_on_line = line_seg.p0 + line_vec * projection;
    return closest_point_on_line;
  }

  // Takes in global plan, which is in map frame.
  // Takes in r_loc, which is the car's position in map frame
  // Fills in closest_seg_idx with the index in global_plan of the segment closest to the car
  // Fills in closest_point with the coords of the closest point to the car on the line segment closest to the car
  // Returns true if the car is close enough to any segment, false otherwise
  bool Navigation::ValidatePlan(const std::vector<Vector2f> global_plan, const Eigen::Vector2f r_loc, int *closest_waypoint_idx)
  {
    float MIN_CLOSEST_DIST = 2;
    visualization::DrawArc(Vector2f(0, 0), MIN_CLOSEST_DIST, 0, 2 * 3.14, 0x00FF00, local_viz_msg_);

    for (int i = global_plan.size() - 1; i >= 0; i--)
    {
      float dist_to_waypoint = (global_plan[i] - r_loc).norm();
      geometry::line2f plan_segment(global_plan[i], global_plan[i - 1]);
      // Check if the car is close enough to this segment
      if (dist_to_waypoint < MIN_CLOSEST_DIST)
      {
        *closest_waypoint_idx = i;
        return true;
      }
    }
    return false;
  }

  // Takes in global plan, which is in map frame.
  // Takes in r_loc, which is the car's position in map frame
  // Takes in r_angle, which is the car's angle in map frame
  // Takes in start_waypoint_idx, which is the index for the waypoint in global_plan that that the car should start from
  // Returns a local intermediate waypoint, which is in map frame
  Eigen::Vector2f Navigation::GetCarrot(vector<Vector2f> global_plan, const Eigen::Vector2f r_loc, float r_angle, size_t start_waypoint_idx)
  {
    float CIRCLE_RADIUS = 1.0;
    visualization::DrawArc(Vector2f(0, 0), CIRCLE_RADIUS, 0, 2 * 3.14, 0x0000FF, local_viz_msg_);

    // If no carrot is found just use the final waypoint
    Vector2f carrot_map = global_plan[global_plan.size() - 1];

    // Iterate over each line in the global_plan starting with the first
    for (size_t i = start_waypoint_idx; i < global_plan.size(); i++)
    {
      if ((global_plan[i] - r_loc).norm() < CIRCLE_RADIUS)
      {
        continue;
      }

      Vector2f p_a = global_plan[i];
      Vector2f p_b = global_plan[i - 1];

      float p = r_loc.x();
      float q = r_loc.y();
      float r = CIRCLE_RADIUS;

      float x_1, x_2, y_1, y_2;

      if ((p_a.x() - p_b.x()) < kEpsilon)
      {
        float k = p_a.x();

        float B = -2 * q;
        float C = pow(p, 2) + pow(q, 2) - pow(r, 2) - (2 * k * p) + pow(k, 2);

        y_1 = (-B + sqrt(pow(B, 2) - 4 * C)) / 2;
        y_2 = (-B - sqrt(pow(B, 2) - 4 * C)) / 2;

        x_1 = k;
        x_2 = k;
      }
      else
      {
        float m = (p_a.y() - p_b.y()) / (p_a.x() / p_b.x());
        float c = p_a.y() - (m * p_a.x());

        float A = pow(m, 2) + 1;
        float B = 2 * (m * c - m * q - p);
        float C = pow(q, 2) - pow(r, 2) + pow(p, 2) - (2 * c * q) + pow(c, 2);

        if ((pow(B, 2) - 4 * A * C) <= 0)
        {
          break;
        }

        x_1 = (-B + sqrt(pow(B, 2) - 4 * A * C)) / (2 * A);
        x_2 = (-B - sqrt(pow(B, 2) - 4 * A * C)) / (2 * A);

        y_1 = m * x_1 + c;
        y_2 = m * x_2 + c;
      }

      Vector2f possible_1 = Vector2f(x_1, y_1);
      Vector2f possible_2 = Vector2f(x_2, y_2);

      if ((possible_1 - p_a).norm() < (possible_2 - p_a).norm())
      {
        carrot_map = possible_1;
      }
      else
      {
        carrot_map = possible_2;
      }

      visualization::DrawCross(carrot_map, 0.12, 0xFF0000, global_viz_msg_);
      visualization::DrawCross(Vector2f(x_1, y_1), 0.1, 0x00FFFF, global_viz_msg_);
      visualization::DrawCross(Vector2f(x_2, y_2), 0.1, 0x00FFFF, global_viz_msg_);

      visualization::DrawCross(p_a, 0.05, 0x00FFFF, global_viz_msg_);
      visualization::DrawCross(p_b, 0.05, 0x00FFFF, global_viz_msg_);

      break;
    }

    // If the above math blows up for some unforseen reason, this allows the robot to keep going instead of halting
    if (std::isnan(carrot_map.x()) || std::isnan(carrot_map.y()))
    {
      carrot_map = global_plan[start_waypoint_idx + 1];
    }
    return carrot_map;
  }

  void Navigation::ObstacleDetector()
  {
    Eigen::Rotation2Df r_map_to_baselink(robot_angle_);
    Eigen::Matrix2f R = r_map_to_baselink.toRotationMatrix();
    Eigen::Matrix3f T_map_to_baselink;
    T_map_to_baselink << R(0, 0), R(0, 1), robot_loc_.x(),
        R(1, 0), R(1, 1), robot_loc_.y(),
        0, 0, 1;

    // // Create an obstacle based on how many points fall within the cell
    int point_count_threshold = 50;
    float discrete_multiplier = 1.0 / FLAGS_resolution; // makes it discrete

    std::map<std::pair<int, int>, bool> unseen_obstacle_grid;
    for (const auto &cell : obstacle_grid_counts)
    {
      const auto &key = cell.first; // Extracting the key
      unseen_obstacle_grid[key] = true;
    }

    for (Vector2f point : point_cloud_)
    {
      Eigen::Vector3f point_to_transform(point.x(), point.y(), 1.0f);
      Eigen::Vector3f transformed_point = T_map_to_baselink * point_to_transform;
      Eigen::Vector2f point_in_correct_frame(transformed_point[0], transformed_point[1]);
      int cellX = std::round(point_in_correct_frame.x() * discrete_multiplier);
      int cellY = std::round(point_in_correct_frame.y() * discrete_multiplier);
      std::pair<int, int> cell = std::make_pair(cellX, cellY);

      // Increment count for the cell
      obstacle_grid_counts[cell]++;
      if (unseen_obstacle_grid.find(cell) == unseen_obstacle_grid.end())
      {
        unseen_obstacle_grid.erase(cell);
      }

      // Check if the count exceeds the threshold
      if (obstacle_grid_counts[cell] > point_count_threshold)
      {
        obstacle_occupancy_grid[cell] = true;
        // check if same cell as anything in global plan
        if (global_plan_grid.find(cell) != global_plan_grid.end())
        {
          // find last waypoint and use it as goal
          int n_waypoints = waypoints.size();
          if (n_waypoints > 0)
          {
            nav_goal_loc_ = waypoints[n_waypoints - 1];
            SetNavGoal(nav_goal_loc_, robot_angle_);
            break;
          }
        }
      }

      std::vector<std::pair<int, int>> cells_to_erase;

      for (const auto &cell : unseen_obstacle_grid)
      {
        const auto &key = cell.first; // Extracting the key
        obstacle_grid_counts[key] -= 10;
        if (obstacle_grid_counts[key] <= point_count_threshold)
        {
          cells_to_erase.push_back(key);
        }
        if (obstacle_grid_counts[key] <= 0)
        {
          obstacle_grid_counts.erase(key);
        }
      }

      for (const auto &cell : cells_to_erase)
      {
        unseen_obstacle_grid.erase(cell);
      }

      // visualize the obstacle occupancy grid, it gets laggy real quick
      // for (const auto& cell: obstacle_occupancy_grid) {
      //       visualization::DrawCross(Vector2f(cell.first.first / discrete_multiplier,
      //                                         cell.first.second / discrete_multiplier), 0.05, 0x000000, global_viz_msg_);
      //       // printf("Cell: (%d, %d)\n", cell.first.first, cell.first.second);
      // }
    }
  }

  void Navigation::Run()
  {
    // This function gets called 20 times a second to form the control loop.

    // Clear previous visualizations.
    visualization::ClearVisualizationMsg(local_viz_msg_);
    visualization::ClearVisualizationMsg(global_viz_msg_);

    // If odometry has not been initialized, we can"t do anything.
    if (!odom_initialized_)
      return;

    // Determine if we already reached a waypoint
    if (waypoints.size() > 0)
    {
      for (unsigned int i = 0; i < waypoints.size(); i++)
      {
        Eigen::Vector2f waypoint = waypoints[i];
        float distance = (waypoint - robot_loc_).norm();
        if (distance < 0.5)
        {
          // then we start from the next waypoint and erase the others
          waypoints.erase(waypoints.begin(), waypoints.begin() + i + 1);
          if (waypoints.size() > 0)
          {
            nav_goal_loc_ = waypoints[0];
          }
          else
          {
            nav_goal_loc_ = robot_loc_;
          }
          break;
        }
      }

      int closest_waypoint_idx = 0;
      bool is_plan_valid = ValidatePlan(waypoints, robot_loc_, &closest_waypoint_idx);
      if (!is_plan_valid)
      {
        if (waypoints.size() > 0)
        {
          nav_goal_loc_ = waypoints[waypoints.size() - 1];
          SetNavGoal(nav_goal_loc_, nav_goal_angle_);
        }
      }
      Vector2f carrot;
      if (waypoints.size() > 1)
      {
        carrot = GetCarrot(waypoints, robot_loc_, robot_angle_, closest_waypoint_idx);
      }
      else
      {
        carrot = waypoints[waypoints.size() - 1];
      }
      nav_goal_loc_ = carrot;
    }
    else
    {
      nav_goal_loc_ = robot_loc_;
    }

    // Draw waypoints from A*
    vector<Eigen::Vector2f>::iterator iter;
    for (iter = waypoints.begin(); iter != waypoints.end(); iter++)
    {
      Eigen::Vector2f waypoint = *iter;
      visualization::DrawCross(waypoint, 0.15, 0x00FF00, global_viz_msg_);
    }
    ObstacleDetector(); // for the global planner

    PathOption p;
    p.free_path_length = 1.0;
    p.curvature = 1.0;
    p.clearance = 1.0;
    // OneDTOC(p);
    LocalObstacleAvoidance(); // for the local planner

    viz_pub_.publish(local_viz_msg_);
    viz_pub_.publish(global_viz_msg_);
  }
} // namespace navigation
