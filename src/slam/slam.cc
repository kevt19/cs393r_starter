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
\file    slam.cc
\brief   SLAM Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
// #include <map>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "simple_queue.h"
#include "slam.h"

#include "vector_map/vector_map.h"

using namespace gtsam;

using namespace math_util;
using Eigen::Affine2f;
using Eigen::Rotation2Df;
using Eigen::Translation2f;
using Eigen::Vector2d;
using Eigen::Vector2f;
using Eigen::Vector4d;
using Eigen::Vector2i;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;

DEFINE_int32(laserInterval, 20, "Number of lasers to use.");
DEFINE_int32(nNodesBeforeSLAM, 20, "Number of nodes to add to gtsam before calling optimize.");
DEFINE_double(slam_dist_threshold, 0.5, "Position threshold for SLAM.");
DEFINE_double(slam_angle_threshold, 30.0, "Angle threshold for SLAM.");

DEFINE_double(slam_min_range, 0.01, "Minimum range to keep a laser reading.");
DEFINE_double(slam_max_range, 10.0, "Maximum range to keep a laser reading.");

DEFINE_int32(slam_num_poses, 100, "Number of poses to keep for SLAM Pose Graph optimization.");
DEFINE_int32(scan_match_timesteps, 1, "Number of previous poses / scans to optimize current pose for.");

DEFINE_double(raster_high_resolution, 0.2, "Resolution to rasterize the map to.");
DEFINE_double(raster_low_resolution, 1.0, "Resolution to rasterize the map to.");
DEFINE_double(raster_map_gaussian_sigma, 0.5, "Sigma for rasterized map.");
DEFINE_double(raster_min_log_prob, -5.0, "Minimum log probability for map point."); // -3.0 is equivalent to 0.001 p
DEFINE_double(maxMapDistance, 1.0, "Maximum distance to consider for log probabilities for map point.");

DEFINE_double(sigma_x, 0.5, "Sigma for x in motion model.");
DEFINE_double(sigma_y, 0.5, "Sigma for y in motion model.");
DEFINE_double(sigma_theta, 0.5, "Sigma for theta in motion model.");

DEFINE_double(incrementAngle, 2.0, "Increment in angle for motion model.");
DEFINE_double(VoxelAngleSize, 30.0, "Maximum size to consider for angle away.");
DEFINE_double(VoxelDistSize, 0.75, "Maximum size to consider for distance away.");

namespace slam 
{
  // constructor
  SLAM::SLAM() 
  {
    prev_odom_loc_ = Eigen::Vector2d(0.0d, 0.0d);
    prev_odom_angle_ = 0.0d;
    ready_to_csm_ = false;
    odom_initialized_ = false;

    // constants
    raster_map_gaussian_sigma_constant = 2*pow(FLAGS_raster_map_gaussian_sigma, 2);
    log_prob_x_constant = (2*pow(FLAGS_sigma_x, 2));
    log_prob_y_constant = (2*pow(FLAGS_sigma_y, 2));
    log_prob_theta_constant = (2*pow(FLAGS_sigma_theta, 2));
    log_prob_motion_constant = -0.5 * log(2.0 * M_PI * pow(FLAGS_sigma_x, 2) * pow(FLAGS_sigma_y, 2) * pow(FLAGS_sigma_theta, 2));

  }


void SLAM::GetPose(Eigen::Vector2d* loc, double* angle) const {
  // Return the latest pose estimate of the robot.
  *loc = Vector2d(0.0d, 0.0d);
  *angle = 0.0f;
}

std::map<std::pair<int,int>, double> SLAM::BuildLowResRasterMapFromHighRes(std::map<std::pair<int,int>, double> high_res_raster_map) 
{
  std::map<std::pair<int,int>, double> low_res_raster_map;
  // build low res map
  // max pool the high res map
  for (const auto& entry : high_res_raster_map) 
  {
    int x = entry.first.first;
    int y = entry.first.second;
    double log_prob = entry.second;
    double ratio = FLAGS_raster_high_resolution / FLAGS_raster_low_resolution;
    int low_res_lookup_x = x * ratio;
    int low_res_lookup_y = y * ratio;
    std::pair<int,int> low_res_lookup_key = std::make_pair(low_res_lookup_x, low_res_lookup_y);
    if (low_res_raster_map.find(low_res_lookup_key) == low_res_raster_map.end()) {
      low_res_raster_map[low_res_lookup_key] = log_prob;
    } else {
      low_res_raster_map[low_res_lookup_key] = std::max(low_res_raster_map[low_res_lookup_key], log_prob);
    }
  }
  return low_res_raster_map;
}

std::map<std::pair<int,int>, double> SLAM::BuildHighResRasterMapFromPoints(const std::vector<Eigen::Vector2d> &alignedPoints)
{
  std::map<std::pair<int,int>, double> high_res_raster_map;
  // iterate over points, compute log probs and lookup values for map
  for (const Eigen::Vector2d& point : alignedPoints) 
  {
    // build grid around x and y
    double start_x = point.x();
    double start_y = point.y();

    // Vector to store the Eigen vectors
    std::vector<Eigen::Vector3d> probabilities;

    for (double x = start_x - FLAGS_maxMapDistance; x <= start_x + FLAGS_maxMapDistance; x += FLAGS_raster_high_resolution) 
      {
        for (double y = start_y - FLAGS_maxMapDistance; y <= start_y + FLAGS_maxMapDistance; y += FLAGS_raster_high_resolution) 
          {
            Eigen::Vector2d vec(x, y);
            double logProbX = -pow((vec[0] - start_x), 2) / raster_map_gaussian_sigma_constant;
            double logProbY = -pow((vec[1] - start_y), 2) / raster_map_gaussian_sigma_constant;
            double logProb = logProbX + logProbY;
            logProb = std::max(logProb, FLAGS_raster_min_log_prob); // min probability

            // get lookup value
            int lookup_x = x / FLAGS_raster_high_resolution;
            int lookup_y = y / FLAGS_raster_high_resolution;
            // check if we already have a value and use max, otherwise just use the value
            std::pair<int,int> lookup_key = std::make_pair(lookup_x, lookup_y);
            if (high_res_raster_map.find(lookup_key) == high_res_raster_map.end()) {
              high_res_raster_map[lookup_key] = logProb;
            } else {
              high_res_raster_map[lookup_key] = std::max(high_res_raster_map[lookup_key], logProb);
            }
          }
      }
  }
  return high_res_raster_map;
}

std::map<std::pair<int,int>, double> SLAM::BuildHighResRasterMapFromMap(const VectorMap& map) {
  // Rasterize the map into a grid map with log probabilities
  // create points from the lines
  std::vector<Eigen::Vector2d> points;

  vector<geometry::line2f> all_map_lines;
  float max_map_range = 10000.0;
  Eigen::Vector2f loc = Eigen::Vector2f(0.0f, 0.0f);
  map.GetSceneLines(loc, max_map_range, &all_map_lines);
  int all_map_lines_size = all_map_lines.size();

  for (int i = 0; i < all_map_lines_size; i++)
  {
    Eigen::Vector2f p0 = all_map_lines[i].p0;
    Eigen::Vector2f p1 = all_map_lines[i].p1;
    double p0_x = p0.x();
    double p0_y = p0.y();
    double p1_x = p1.x();
    double p1_y = p1.y();
    
    for (double p = p0_x; p < p1_x; p += FLAGS_raster_high_resolution) {
      for (double q = p0_y; q < p1_y; q += FLAGS_raster_high_resolution) {
        points.push_back(Eigen::Vector2d(p, q));
      }
    }
  }
  return BuildHighResRasterMapFromPoints(points);
}

vector<Eigen::Vector2d> SLAM::AlignPointCloud(const vector<Eigen::Vector2d>& point_cloud,
                                  const Vector2d& optimized_loc,
                                  const double optimized_angle) {
  vector<Eigen::Vector2d> aligned_point_cloud;
  Eigen::Rotation2Dd r_optimized_pose_to_first_pose(1.0 * optimized_angle);
  Eigen::Matrix2d R = r_optimized_pose_to_first_pose.toRotationMatrix();
  Eigen::Vector2d translation = optimized_loc;

  for (const Eigen::Vector2d& point : point_cloud) {
    double x = point.x();
    double y = point.y();

    Eigen::Vector2d point_to_transform(x, y);
    point_to_transform = R * point_to_transform;
    Eigen::Vector2d transformed_point = point_to_transform + translation;
    aligned_point_cloud.push_back(transformed_point);
  }
  return aligned_point_cloud;
}

std::vector<std::pair<Eigen::Vector3d, Eigen::MatrixXd>> SLAM::CorrelativeScanMatching(const vector<Eigen::Vector2d>& point_cloud,
                                   const Vector2d& odom_loc, 
                                   const double odom_angle) 
{
  std::vector <std::pair<Eigen::Vector3d, Eigen::MatrixXd>> bestPosesWithVariances;
  int min_i = std::max(0, (int) optimizedPoses_.size() - FLAGS_scan_match_timesteps);
  for (int i = optimizedPoses_.size() - 1; i >= min_i; i--)
  {
    Eigen::Vector2d prev_loc = Eigen::Vector2d(optimizedPoses_[i].x(), optimizedPoses_[i].y());
    double prev_angle = optimizedPoses_[i].z();
    std::map<std::pair<int,int>, double> high_res_raster_map = high_res_raster_maps_[i];
    std::map<std::pair<int,int>, double> low_res_raster_map = low_res_raster_maps_[i];
    std::pair<Eigen::Vector3d, Eigen::MatrixXd> bestPoseWithVar = SingleCorrelativeScanMatching(point_cloud, odom_loc, odom_angle, prev_loc, prev_angle, low_res_raster_map, high_res_raster_map);
    Eigen::Vector3d bestPose = bestPoseWithVar.first;
    Eigen::MatrixXd bestCovariance = bestPoseWithVar.second;
    bestPosesWithVariances.push_back(std::make_pair(bestPose, bestCovariance));
  }
  return bestPosesWithVariances;
}

std::pair<Eigen::Vector3d, Eigen::MatrixXd> SLAM::SingleCorrelativeScanMatching(const vector<Eigen::Vector2d>& point_cloud,
                                   const Vector2d& odom_loc, 
                                   const double odom_angle,
                                   const Eigen::Vector2d& prev_loc,
                                   const double prev_angle,
                                   std::map<std::pair<int,int>, double> low_res_raster_map,
                                   std::map<std::pair<int,int>, double> high_res_raster_map) {
  // Implement correlative scan matching to find the best pose given the
  // previous pose and the current laser scan.

  double startX = prev_loc.x();
  double minX = startX - FLAGS_VoxelDistSize;
  double maxX = startX + FLAGS_VoxelDistSize;

  double startY = prev_loc.y();
  double minY = startY - FLAGS_VoxelDistSize;
  double maxY = startY + FLAGS_VoxelDistSize;
 
  double prevAngleDeg = prev_angle * 180.0 / M_PI;
  double minAngle = prevAngleDeg - FLAGS_VoxelAngleSize;
  // minAngle = prevAngleDeg + FLAGS_VoxelAngleSize; // DEBUG STATEMENT: TODO: REMOVE
  double maxAngle = prevAngleDeg + FLAGS_VoxelAngleSize;

  // convert to radians
  minAngle = minAngle * M_PI / 180.0;
  maxAngle = maxAngle * M_PI / 180.0;
  double incrementAngleRad = FLAGS_incrementAngle * M_PI / 180.0;

  SimpleQueue<Eigen::Vector4d, double> bestLowResVoxels;

  // initialize K, u, s for uncertainty estimate later
  Eigen::MatrixXd K = Eigen::MatrixXd::Zero(3, 3);
  Eigen::Vector3d u = Eigen::Vector3d(0.0d, 0.0d, 0.0d);
  double s = 0.0d;
  double term_idx = 0.0; // saved so we can extract from diagnal later

  // low res voxel grid search
  for (double angle = minAngle; angle <= maxAngle; angle += incrementAngleRad) 
  {
    double delta_angle = angle - prev_angle;
    double log_prob_theta = -pow(delta_angle, 2) / log_prob_theta_constant;
    Eigen::Rotation2Dd r_optimized_pose_to_first_pose(1.0 * angle);
    Eigen::Matrix2d R = r_optimized_pose_to_first_pose.toRotationMatrix();

    // rotated points optimized computation
    vector<Eigen::Vector2d> rotated_points;
    for (const Eigen::Vector2d& point : point_cloud)
    {
      Eigen::Vector2d point_to_transform = point;
      point_to_transform = R * point_to_transform;
      Eigen::Vector2d rotated_point = point_to_transform;
      rotated_points.push_back(rotated_point);
    }

    // coarse search over x
    for (double x = minX; x <= maxX; x += FLAGS_raster_low_resolution) 
    {
      double delta_x = x - prev_loc.x();
      double log_prob_x = -pow(delta_x, 2) / log_prob_x_constant; 

      // coarse search over y
      for (double y = minY; y <= maxY; y += FLAGS_raster_low_resolution) 
      {
        double delta_y = y - prev_loc.y();
        double log_prob_y = -pow(delta_y, 2) / log_prob_y_constant; 
        double log_prob_motion = log_prob_x + log_prob_y + log_prob_theta;
        Eigen::Vector2d translation = Eigen::Vector2d(x, y);

        // check low-res map
        double log_prob_map = 0;
        for (const Eigen::Vector2d& r_point : rotated_points) 
        {
          Eigen::Vector2d map_point = r_point + translation;
          double map_x = map_point.x();
          double map_y = map_point.y();
          int lookup_x = map_x / FLAGS_raster_low_resolution;
          int lookup_y = map_y / FLAGS_raster_low_resolution;
          if (low_res_raster_map.find(std::make_pair(lookup_x, lookup_y)) != low_res_raster_map.end()) {
            log_prob_map += low_res_raster_map[std::make_pair(lookup_x, lookup_y)];
          }
          else {
            log_prob_map += FLAGS_raster_min_log_prob; // since we didn't find anything, min log probability
          }
        }

        double log_prob = log_prob_motion_constant + log_prob_motion + log_prob_map;
        log_prob = std::max(log_prob, -100000.0); // numerical stability
        double prob = exp(log_prob);
        Eigen::Vector3d features = Eigen::Vector3d(x, y, angle);
        Eigen::Matrix3d currentKTerm = features * features.transpose() * prob;
        K += currentKTerm;
        Eigen::Vector3d currentUTerm = features * prob;
        u += currentUTerm;
        s += prob;
        bestLowResVoxels.Push(Eigen::Vector4d(x, y, angle, term_idx), log_prob);
        term_idx += 1.0d;
      }
    }
  }

  Eigen::MatrixXd CovPoses = K / s - 1 / pow(s,2) * (u * u.transpose());

  double bestLogProb = -100000.0;
  double prev_angle_d = prev_angle;
  Eigen::Vector3d bestPose = Eigen::Vector3d(startX, startY, prev_angle_d);

  // now do high res search with priority queue
  while (!bestLowResVoxels.Empty())
  {
    std::pair<Eigen::Vector4d, double> poppedVal = bestLowResVoxels.PopWithPriority();
    Eigen::Vector4d bestLowResVoxelWithIdx = poppedVal.first;
    Eigen::Vector3d bestLowResVoxel = Eigen::Vector3d(bestLowResVoxelWithIdx.x(), bestLowResVoxelWithIdx.y(), bestLowResVoxelWithIdx.z());
    double log_prob = poppedVal.second;

    if (log_prob < bestLogProb) {
      std::pair<Eigen::Vector3d, Eigen::MatrixXd> bestPoseWithVar = std::make_pair(bestPose, CovPoses);
      return bestPoseWithVar;
    }

    double lowResX = bestLowResVoxel.x();
    double lowResY = bestLowResVoxel.y();
    double angle = bestLowResVoxel.z();
    double minX = lowResX;
    double minY = lowResY;
    double maxX = lowResX + FLAGS_raster_low_resolution;
    double maxY = lowResY + FLAGS_raster_low_resolution;

    double delta_angle = angle - prev_angle;
    double log_prob_theta = -pow((delta_angle), 2) / (2*pow(FLAGS_sigma_theta, 2)); // TODO fix speedup
    Eigen::Rotation2Dd r_optimized_pose_to_first_pose(1.0 * angle);
    Eigen::Matrix2d R = r_optimized_pose_to_first_pose.toRotationMatrix();

    // rotated points optimized computation
    vector<Eigen::Vector2d> rotated_points;
    for (const Eigen::Vector2d& point : point_cloud) 
    {
      double point_x = point.x();
      double point_y = point.y();
      Eigen::Vector2d point_to_transform(point_x, point_y);
      point_to_transform = R * point_to_transform;
      Eigen::Vector2d rotated_point = point_to_transform;
      rotated_points.push_back(rotated_point);
    }

    for (double x = minX; x <= maxX; x += FLAGS_raster_high_resolution) 
    {
      double delta_x = x - prev_loc.x();
      double log_prob_x = -pow((delta_x), 2) / (2*pow(FLAGS_sigma_x, 2)); 

      for (double y = minY; y <= maxY; y += FLAGS_raster_high_resolution) 
      {
        double delta_y = y - prev_loc.y();
        double log_prob_y = -pow(delta_y, 2) / (2*pow(FLAGS_sigma_y, 2)); 
        double log_prob_motion = log_prob_x + log_prob_y + log_prob_theta;
        Eigen::Vector2d translation = Eigen::Vector2d(x, y);

        // check low-res map
        double high_res_log_prob_map = 0.0;
        for (const Eigen::Vector2d& r_point : rotated_points) 
        {
          Eigen::Vector2d map_point = r_point + translation;
          double map_x = map_point.x();
          double map_y = map_point.y();
          int lookup_x = map_x / FLAGS_raster_high_resolution;
          int lookup_y = map_y / FLAGS_raster_high_resolution;
          if (high_res_raster_map.find(std::make_pair(lookup_x, lookup_y)) != high_res_raster_map.end()) {
            high_res_log_prob_map += high_res_raster_map[std::make_pair(lookup_x, lookup_y)];
          }
          else {
            high_res_log_prob_map += FLAGS_raster_min_log_prob; // since we didn't find anything, min log probability
          }
        }

        double log_prob = log_prob_motion_constant + log_prob_motion + high_res_log_prob_map;
        if (log_prob > bestLogProb) {
          // printf("Found better log prob: %f\n", log_prob);
          // printf("Log prob decomposed into: %f, %f, %f, %f\n", log_prob_motion_constant, log_prob_motion, high_res_log_prob_map, log_prob);
          bestLogProb = log_prob;
          bestPose = Eigen::Vector3d(x, y, angle);
        }
      }
    }
  }
  std::pair<Eigen::Vector3d, Eigen::MatrixXd> bestPoseWithVar = std::make_pair(bestPose, CovPoses);
  return bestPoseWithVar;
}

  void SLAM::ObserveLaser(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max) {
    // A new laser scan has been observed. Decide whether to add it as a pose
    // for SLAM. If decided to add, align it to the scan from the last saved pose,
    // and save both the scan and the optimized pose.

    size_t num_ranges = ranges.size();

    double angle_increment = (angle_max - angle_min) / (num_ranges - 1);
    std::vector<Eigen::Vector2d> point_cloud;
    for (size_t i = 0; i < num_ranges; i++)
    {
      if (i % FLAGS_laserInterval != 0) {
        continue;
      }
      double range = ranges[i];
      if (FLAGS_slam_min_range < range && range < FLAGS_slam_max_range && range_min < range && range < range_max) {
        double angle = angle_min + i * angle_increment;
        Eigen::Vector2d point;
        point = Eigen::Vector2d(range*cos(angle), range*sin(angle));
        // point = range *  Vector2d(cos(angle), sin(angle)) + laser_location; // ignoring lidar scanner loc for now
        point_cloud.push_back(point); // Add point to the cloud
      }
    }
    
    // if this is the first scan, let's save it and return
    int num_scans = alignedPointsOverPoses_.size();
    if (ready_to_csm_ && num_scans == 0) {
      double prev_loc_x = prev_odom_loc_.x();
      double prev_loc_y = prev_odom_loc_.y();
      double prec_angle = prev_odom_angle_;
      Eigen::Vector3d previous_pose = Eigen::Vector3d(prev_loc_x, prev_loc_y, prec_angle);
      optimizedPoses_.push_back(previous_pose);
      pointClouds_.push_back(point_cloud);
      vector<Eigen::Vector2d> aligned_point_cloud = AlignPointCloud(point_cloud, prev_odom_loc_, prev_odom_angle_);
      high_res_raster_maps_.push_back(BuildHighResRasterMapFromPoints(aligned_point_cloud));
      low_res_raster_maps_.push_back(BuildLowResRasterMapFromHighRes(high_res_raster_maps_[0]));
      alignedPointsOverPoses_.push_back(aligned_point_cloud);
      Eigen::MatrixXd initial_cov = Eigen::MatrixXd::Zero(3, 3);
      optimizedPosesVariances_.push_back(initial_cov);
      ready_to_csm_ = false; 
      return;
    }

    if (!ready_to_csm_) {
      return;
    }

    // Get the optimized pose
    Eigen::Vector3d prev_optimized_pose = optimizedPoses_[num_scans - 1];
    Eigen::Vector2d prev_optimized_loc = Eigen::Vector2d(prev_optimized_pose.x(), prev_optimized_pose.y());
    double prev_optimized_angle = optimizedPoses_[num_scans - 1].z();
    std::vector<std::pair<Eigen::Vector3d, Eigen::MatrixXd>> all_optimized_pose_and_var = CorrelativeScanMatching(point_cloud, prev_optimized_loc, prev_optimized_angle);
    
    int num_new_optimized_poses = all_optimized_pose_and_var.size();

    for (int i = 0; i < num_new_optimized_poses; i++) {
      Eigen::Vector3d optimized_pose = all_optimized_pose_and_var[i].first;
      Eigen::MatrixXd optimized_var = all_optimized_pose_and_var[i].second;
      Eigen::Vector2d optimized_loc = Eigen::Vector2d(optimized_pose.x(), optimized_pose.y());  
      double optimized_angle = optimized_pose.z();
      // update point cloud
      std::vector<Eigen::Vector2d> aligned_point_cloud = AlignPointCloud(point_cloud, optimized_loc, optimized_angle);
      std::map<std::pair<int,int>, double> high_res_raster_map = BuildHighResRasterMapFromPoints(aligned_point_cloud);
      std::map<std::pair<int,int>, double> low_res_raster_map = BuildLowResRasterMapFromHighRes(high_res_raster_map);

      // if we have too many poses, remove the oldest one
      if (num_scans >= FLAGS_slam_num_poses) {
        pointClouds_.erase(pointClouds_.begin());
        alignedPointsOverPoses_.erase(alignedPointsOverPoses_.begin());
        optimizedPoses_.erase(optimizedPoses_.begin());
        high_res_raster_maps_.erase(high_res_raster_maps_.begin());
        low_res_raster_maps_.erase(low_res_raster_maps_.begin());
        optimizedPosesVariances_.erase(optimizedPosesVariances_.begin());
      }

      pointClouds_.push_back(point_cloud);
      alignedPointsOverPoses_.push_back(aligned_point_cloud);
      optimizedPoses_.push_back(optimized_pose);
      high_res_raster_maps_.push_back(high_res_raster_map);
      low_res_raster_maps_.push_back(low_res_raster_map);
      optimizedPosesVariances_.push_back(optimized_var);
      ready_to_csm_ = false; 

      // add to factor graph
      // printf("sTart\n");
      Eigen::Vector3d optimized_var_diag = optimized_var.diagonal();
      Vector3 optimized_std_diag = Vector3(optimized_var_diag.x(), optimized_var_diag.y(), optimized_var_diag.z());
      noiseModel::Diagonal::shared_ptr stdForOptimizedPose = noiseModel::Diagonal::Sigmas(optimized_std_diag);
      Eigen::Vector3d priorPose = optimizedPoses_[num_scans - 1 - i];
      // printf("fo\n");
      Eigen::Vector3d deltaPose = optimized_pose - priorPose;
      Pose2 deltaPoseForGraph = Pose2(deltaPose.x(), deltaPose.y(), deltaPose.z());
      Pose2 optimizedPoseForGraph = Pose2(optimized_pose.x(), optimized_pose.y(), optimized_pose.z());
      // printf("before adding\n");
      graph_.add(BetweenFactor<Pose2>(nNodesInGraph - i, nNodesInGraph + 1, deltaPoseForGraph, stdForOptimizedPose));
      initialGuesses_.insert(nNodesInGraph + 1, optimizedPoseForGraph);
      // printf("after inserting\n");
      nNodesInGraph += 1;
    }

    if (nNodesInGraph > 10 && nNodesInGraph % FLAGS_nNodesBeforeSLAM == 0) {
      PoseGraphOptimization();

    }
  }

  void SLAM::PoseGraphOptimization() {
    // Optimize the graph using Levenberg-Marquardt optimization
    LevenbergMarquardtOptimizer optimizer(graph_, initialGuesses_);
    // printf("before optimizing\n");
    Values result = optimizer.optimize();
    for (int i = nNodesInGraph - 1; i >= 0; i--)
    {
      double x = result.at<Pose2>(i + 1).x();
      double y = result.at<Pose2>(i + 1).y();
      double theta = result.at<Pose2>(i + 1).theta();
      // printf("initial guess %f, %f", initialGuesses_.at<Pose2>(i).x(), initialGuesses_.at<Pose2>(i).y());
      // printf("optimized loc %f, %f", optimizedPoses_[i].x(), optimizedPoses_[i].y());
      // printf("updated optimized loc %f, %f", x, y);

      Eigen::Vector3d updatedOptimizedPose = Eigen::Vector3d(x, y, theta);
      Eigen::Vector2d updatedOptimizedLoc = Eigen::Vector2d(x, y);
      
      // update our vector
      optimizedPoses_[i] = updatedOptimizedPose;
      // update aligned point clouds
      
      std::vector<Eigen::Vector2d> updatedAlignedPointCloud = AlignPointCloud(pointClouds_[i], updatedOptimizedLoc, theta);
      alignedPointsOverPoses_[i] = updatedAlignedPointCloud; 
    }

    // update optimized poses and aligned pcs

    // printf("after optimizing\n");
  }

void SLAM::ObserveOdometry(const Vector2d& odom_loc, const double odom_angle) {
  if (!odom_initialized_) {
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;    
    odom_initialized_ = true;
    ready_to_csm_ = true;

    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.0, 0.0, 0.0));
    graph_.add(PriorFactor<Pose2>(1, Pose2(0, 0, 0), priorNoise));
    initialGuesses_.insert(1, Pose2(odom_loc.x(), odom_loc.y(), odom_angle));
    return;
  }

  double delta_odom_x = odom_loc.x() - prev_odom_loc_.x();
  double delta_odom_y = odom_loc.y() - prev_odom_loc_.y();
  double delta_odom_angle = RadToDeg(odom_angle - prev_odom_angle_);
  double delta_odom_dist = sqrt(pow(delta_odom_x, 2) + pow(delta_odom_y, 2));
  if (delta_odom_dist < FLAGS_slam_dist_threshold && fabs(delta_odom_angle) < FLAGS_slam_angle_threshold) {
    return;
  }
  ready_to_csm_ = true;
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;
}

vector<Vector2f> SLAM::GetMap() {
  vector<Vector2f> map;
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.

  // convert w.r.t first pose
  int num_scans = alignedPointsOverPoses_.size();
  if (num_scans == 0) {
    return map;
  }

  for (int i = 0; i < num_scans; i++) {
    vector<Eigen::Vector2d> point_cloud = alignedPointsOverPoses_[i];
    for (const Eigen::Vector2d& point : point_cloud) {
      float x = point.x();
      float y = point.y();
      map.push_back(Eigen::Vector2f(x, y));
    }
  }
  return map;
}

}  // namespace slam
