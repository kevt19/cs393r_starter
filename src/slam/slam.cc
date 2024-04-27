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


// #include <gtsam/geometry/Pose2.h>
// #include <gtsam/nonlinear/NonlinearFactorGraph.h>
// #include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
// #include <gtsam/nonlinear/Values.h>

// using namespace gtsam;

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

DEFINE_double(slam_dist_threshold, 0.5, "Position threshold for SLAM.");
DEFINE_double(slam_angle_threshold, 30, "Angle threshold for SLAM."); // in deg, need to check for consistency

DEFINE_double(slam_min_range, 0.1, "Minimum range to keep a laser reading.");
DEFINE_double(slam_max_range, 10.0, "Maximum range to keep a laser reading.");

DEFINE_int32(slam_num_poses, 10, "Number of poses to keep for SLAM Pose Graph optimization.");
DEFINE_int32(scan_match_timesteps, 1, "Number of previous poses / scans to optimize current pose for.");

DEFINE_double(raster_high_resolution, 0.05, "Resolution to rasterize the map to.");
DEFINE_double(raster_low_resolution, 0.5, "Resolution to rasterize the map to.");
DEFINE_double(raster_map_gaussian_sigma, 1.0, "Sigma for rasterized map.");

DEFINE_double(sigma_x, 0.05, "Sigma for x in motion model.");
DEFINE_double(sigma_y, 0.05, "Sigma for y in motion model.");
DEFINE_double(sigma_theta, 0.1, "Sigma for theta in motion model.");

DEFINE_double(incrementAngle, 1, "Increment in angle for motion model.");
DEFINE_double(VoxelAngleSize, 60.0, "Maximum size to consider for angle away.");

DEFINE_double(VoxelDistSize, 0.75, "Maximum size to consider for distance away."); // likely correlated with slam_dist_threshold

DEFINE_double(maxMapDistance, 1.0, "Maximum distance to consider for log probabilities for map point.");

namespace slam 
{
  // constructor
  SLAM::SLAM() 
  {
    prev_odom_loc_ = Eigen::Vector2d(0.0d, 0.0d);
    prev_odom_angle_ = 0.0d;
    ready_to_csm_ = false;
    odom_initialized_ = false;
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
    if (low_res_raster_map.find(std::make_pair(low_res_lookup_x, low_res_lookup_y)) == low_res_raster_map.end()) {
      low_res_raster_map[std::make_pair(low_res_lookup_x, low_res_lookup_y)] = log_prob;
    } else {
      low_res_raster_map[std::make_pair(low_res_lookup_x, low_res_lookup_y)] = std::max(low_res_raster_map[std::make_pair(low_res_lookup_x, low_res_lookup_y)], log_prob);
    }
  }
  return low_res_raster_map;
}


std::map<std::pair<int,int>, double> SLAM::BuildHighResRasterMapFromPoints(const std::vector<Eigen::Vector2d> &points)
{
  std::map<std::pair<int,int>, double> high_res_raster_map;
  // iterate over points, compute log probs and lookup values for map
  for (const Eigen::Vector2d& point : points) 
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
            double logProbX = -pow((vec[0] - start_x), 2) / (2*pow(FLAGS_raster_map_gaussian_sigma, 2));
            double logProbY = -pow((vec[1] - start_y), 2) / (2*pow(FLAGS_raster_map_gaussian_sigma, 2));
            double logProb = logProbX + logProbY;

            // get lookup value
            int lookup_x = x / FLAGS_raster_high_resolution;
            int lookup_y = y / FLAGS_raster_high_resolution;
            // check if we already have a value and use max, otherwise just use the value
            if (high_res_raster_map.find(std::make_pair(lookup_x, lookup_y)) == high_res_raster_map.end()) {
              high_res_raster_map[std::make_pair(lookup_x, lookup_y)] = logProb;
            } else {
              high_res_raster_map[std::make_pair(lookup_x, lookup_y)] = std::max(high_res_raster_map[std::make_pair(lookup_x, lookup_y)], logProb);
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

vector<Eigen::Vector2d> SLAM::AlignedPointCloud(const vector<Eigen::Vector2d>& point_cloud,
                                  const Vector2d& optimized_loc,
                                  const double optimized_angle) {


    // Eigen::Rotation2Dd r_pose_to_optimized_pose(-1.0 * optimized_angle);
    // 30 deg to rad
    double optimized_angle_rad = optimized_angle * M_PI / 180.0;
    Eigen::Rotation2Dd r_pose_to_optimized_pose(optimized_angle_rad);
    Eigen::Matrix2d R = r_pose_to_optimized_pose.toRotationMatrix();
    Eigen::Matrix3d T_pose_to_optimized_pose;

    // double translationX = -optimized_loc.x();
    // double translationY = -optimized_loc.y();

    double translationX = 0;
    double translationY = 0;

    T_pose_to_optimized_pose << R(0, 0), R(0, 1), 0,
        R(1, 0), R(1, 1), 0,
        0, 0, 1;
    Eigen::Vector3d translation(translationX, translationY, 0.0f);


  vector<Eigen::Vector2d> aligned_point_cloud;
  for (const Eigen::Vector2d& point : point_cloud) 
  {
    double x = point.x();
    double y = point.y();

    printf("x: %f, y: %f\n", x, y);


    Eigen::Vector3d point_to_transform(x, y, 1.0d);

    point_to_transform = point_to_transform - translation;
    Eigen::Vector3d transformed_point = T_pose_to_optimized_pose * point_to_transform;
    double transformed_x = transformed_point.x();
    double transformed_y = transformed_point.y();
    printf("transformed x: %f, transformed y: %f\n", transformed_x, transformed_y);
    Eigen::Vector2d transformed_point_2d = Eigen::Vector2d(transformed_x, transformed_y);
    aligned_point_cloud.push_back(transformed_point_2d);

    // double rotated_x = x * cos(optimized_angle) - y * sin(optimized_angle);
    // double rotated_y = x * sin(optimized_angle) + y * cos(optimized_angle);
    // aligned_point_cloud.push_back(Eigen::Vector2d(rotated_x + optimized_loc.x(), rotated_y + optimized_loc.y()));
  }
  return aligned_point_cloud;
}

Eigen::Vector4d SLAM::CorrelativeScanMatching(const vector<Eigen::Vector2d>& point_cloud,
                                   const Vector2d& odom_loc, 
                                   const double odom_angle) 
{
  // get previous pose and loop backwards through all previous poses
  printf("Top of CSM\n");
  Eigen::Vector3d bestPoseCounter = Eigen::Vector3d(0.0d, 0.0d, 0.0d);
  double bestPoseVarianceCounter = 0.0d;
  int min_i = std::max(0, (int) optimizedPoses_.size() - FLAGS_scan_match_timesteps);
  double n_iters = 0.0;
  for (int i = optimizedPoses_.size() - 1; i >= min_i; i--) 
  {
    printf("making vector with prev loc\n");
    Eigen::Vector2d prev_loc = Eigen::Vector2d(optimizedPoses_[i].x(), optimizedPoses_[i].y());
    double prev_angle = optimizedPoses_[i].z();
    printf("Getting raster map");
    std::map<std::pair<int,int>, double> high_res_raster_map = high_res_raster_maps_[i];
    std::map<std::pair<int,int>, double> low_res_raster_map = low_res_raster_maps_[i];

    printf("Running CSM for i=%d\n", i);
    Eigen::Vector4d bestPoseWithVar = SingleCorrelativeScanMatching(point_cloud, odom_loc, odom_angle, prev_loc, prev_angle, low_res_raster_map, high_res_raster_map);
    printf("setting best pose\n");
    Eigen::Vector3d bestPose = Eigen::Vector3d(bestPoseWithVar.x(), bestPoseWithVar.y(), bestPoseWithVar.z());
    printf("extracting variance\n");
    double bestVariance = bestPoseWithVar.w();
    printf("adding stuff\n");
    bestPoseCounter += bestPose;
    bestPoseVarianceCounter += bestVariance;
    n_iters += 1.0;
  }

  printf("Bottom of CSM\n");

  if (n_iters == 0.0) {
    return Eigen::Vector4d(0.0d, 0.0d, 0.0d, 0.0d); // this should never happen, probably better to create an exception here
  }

  Eigen::Vector3d bestPose = bestPoseCounter / n_iters;
  double bestVariance = bestPoseVarianceCounter / n_iters;
  return Eigen::Vector4d(bestPose.x(), bestPose.y(), bestPose.z(), bestVariance);
}


Eigen::Vector4d SLAM::SingleCorrelativeScanMatching(const vector<Eigen::Vector2d>& point_cloud,
                                   const Vector2d& odom_loc, 
                                   const double odom_angle,
                                   const Eigen::Vector2d& prev_loc,
                                   const double prev_angle,
                                   std::map<std::pair<int,int>, double> low_res_raster_map,
                                   std::map<std::pair<int,int>, double> high_res_raster_map) {
  // Implement correlative scan matching to find the best pose given the
  // previous pose and the current laser scan.

  double startX = prev_loc.x();
  double startY = prev_loc.y();

  double minX = startX - FLAGS_VoxelDistSize;
  double minY = startY - FLAGS_VoxelDistSize;
  double minAngle = prev_angle - FLAGS_VoxelAngleSize;

  double maxX = startX + FLAGS_VoxelDistSize;
  double maxY = startY + FLAGS_VoxelDistSize;
  double maxAngle = prev_angle + FLAGS_VoxelAngleSize;

  printf("Creating queue...\n");

  SimpleQueue<Eigen::Vector4d, double> bestLowResVoxels;

  Eigen::MatrixXd K = Eigen::MatrixXd::Zero(3, 3);
  Eigen::Vector3d u = Eigen::Vector3d(0.0d, 0.0d, 0.0d);
  double s = 0.0d;
  double term_idx = 0;
  double log_prob_x_constant = (2*pow(FLAGS_sigma_x, 2));
  double log_prob_y_constant = (2*pow(FLAGS_sigma_y, 2));
  double log_prob_theta_constant = (2*pow(FLAGS_sigma_theta, 2));


  double log_prob_motion_constant = -0.5 * log(2.0 * M_PI * pow(FLAGS_sigma_x, 2) * pow(FLAGS_sigma_y, 2) * pow(FLAGS_sigma_theta, 2));


  printf("Starting loop for low res...\n");

  // low res voxel grid
  for (double angle = minAngle; angle <= maxAngle; angle += FLAGS_incrementAngle) 
  {
    // TODO: make sure these are consistent with radians/degs throughout code
    // log prob for angle for the motion model
    double log_prob_theta = -pow((angle - odom_angle), 2) / log_prob_theta_constant;

    // rotated points optimized computation
    vector<Eigen::Vector2d> rotated_points;
    printf("Rotating points...\n");
    printf("min x: %f, max x: %f\n", minX, maxX);
    printf("min y: %f, max y: %f\n", minY, maxY);
    for (const Eigen::Vector2d& point : point_cloud)
    {
      double point_x = point.x();
      double point_y = point.y();
      double rotated_x = point_x * cos(angle) - point_y * sin(angle);
      double rotated_y = point_x * sin(angle) + point_y * cos(angle);
      rotated_points.push_back(Eigen::Vector2d(rotated_x, rotated_y));
    }

    // printf("Starting loop for x...\n");
    for (double x = minX; x < maxX; x += FLAGS_raster_low_resolution) 
    {
      double log_prob_x = -pow((x - odom_loc.x()), 2) / log_prob_x_constant; 

      printf("Starting loop for y...\n");
      for (double y = minY; y < maxY; y += FLAGS_raster_low_resolution) 
      {
        // printf("Computing log prob for y=%f\n", y);
        double log_prob_y = -pow((y - odom_loc.y()), 2) / log_prob_y_constant; 
        double log_prob_motion = log_prob_x + log_prob_y + log_prob_theta;
        printf("Computed motion prob for y=%f\n", y);

        Eigen::Vector3d features = Eigen::Vector3d(x, y, angle);

        // check low-res map
        double log_prob_map = 0;
        for (const Eigen::Vector2d& r_point : rotated_points) 
        {
          double map_x = x + r_point.x();
          double map_y = y + r_point.y();
          int lookup_x = map_x / FLAGS_raster_low_resolution;
          int lookup_y = map_y / FLAGS_raster_low_resolution;
          // printf("Looking up x=%d, y=%d\n", lookup_x, lookup_y);
          if (low_res_raster_map.find(std::make_pair(lookup_x, lookup_y)) != low_res_raster_map.end()) {
            log_prob_map += low_res_raster_map[std::make_pair(lookup_x, lookup_y)];
          }
        }

        double log_prob = log_prob_motion_constant + log_prob_motion + log_prob_map;
        // also multiply constant to make it a probability
        printf("computing prob for y=%f\n", y);
        log_prob = std::max(log_prob, -100000.0); // numerical stability
        printf("log_prob: %f\n", log_prob);
        double prob = exp(log_prob);
        printf("prob: %f\n", prob);
        printf("doing some matrix multiply\n");
        Eigen::Matrix3d currentKTerm = features * features.transpose() * prob;
        K += currentKTerm;
        printf("multiply vector by constant\n");
        Eigen::Vector3d currentUTerm = features * prob;
        printf("adding u term\n");
        u += currentUTerm;
        printf("adding prob\n");
        s += prob;
        printf("pushing low res vowel\n");
        bestLowResVoxels.Push(Eigen::Vector4d(x, y, angle, term_idx), log_prob);
        printf("incrementing term idx\n");
        term_idx += 1.0d;
      }
    }
  }

  Eigen::MatrixXd CovPoses = K / s - 1 / pow(s,2) * (u * u.transpose());
  printf("finished covariance poses shit\n");

  double bestLogProb = -100000.0;
  int bestIdx = 0;
  double prev_angle_d = prev_angle;
  Eigen::Vector3d bestPose = Eigen::Vector3d(startX, startY, prev_angle_d);
  printf("starting loop lol\n");

  // now do high res search with priority queue
  while (!bestLowResVoxels.Empty())
  {
    std::pair<Eigen::Vector4d, double> poppedVal = bestLowResVoxels.PopWithPriority();
    Eigen::Vector4d bestLowResVoxelWithIdx = poppedVal.first;
    Eigen::Vector3d bestLowResVoxel = Eigen::Vector3d(bestLowResVoxelWithIdx.x(), bestLowResVoxelWithIdx.y(), bestLowResVoxelWithIdx.z());
    int idx = bestLowResVoxelWithIdx.w();
    double log_prob = poppedVal.second;

    if (log_prob < bestLogProb) {
      // add best cov to what we send back
      // get diagonal i of covariance matrix
      printf("found best\n");

      double bestCov = CovPoses(bestIdx, bestIdx);
      printf("getting cov poses is bad????\n");
      Eigen::Vector4d bestPoseWithVar = Eigen::Vector4d(bestPose.x(), bestPose.y(), bestPose.z(), bestCov);
      printf("returning\n");
      return bestPoseWithVar; // this only happens if the best is the very last one
    }

    // otherwise let's try our high-res search
    double lowResX = bestLowResVoxel.x();
    double lowResY = bestLowResVoxel.y();
    double angle = bestLowResVoxel.z();

    // double minAngle = angle - FLAGS_incrementAngle; // we do high-res search for angle above!
    // double maxAngle = angle + FLAGS_incrementAngle;
    double minX = lowResX;
    double minY = lowResY;
    double maxX = lowResX + FLAGS_raster_low_resolution;
    double maxY = lowResY + FLAGS_raster_low_resolution;

    double log_prob_theta = -pow((angle - odom_angle), 2) / (2*pow(FLAGS_sigma_theta, 2));

    // rotated points optimized computation
    vector<Eigen::Vector2d> rotated_points;
    for (const Eigen::Vector2d& point : point_cloud) 
    {
      double point_x = point.x();
      double point_y = point.y();
      double rotated_x = point_x * cos(angle) - point_y * sin(angle);
      double rotated_y = point_x * sin(angle) + point_y * cos(angle);
      rotated_points.push_back(Eigen::Vector2d(rotated_x, rotated_y));
    }

    for (double x = minX; x <= maxX; x += FLAGS_raster_high_resolution) 
    {
      double log_prob_x = -pow((x - odom_loc.x()), 2) / (2*pow(FLAGS_sigma_x, 2)); 

      for (double y = minY; y <= maxY; y += FLAGS_raster_high_resolution) 
      {
        double log_prob_y = -pow((y - odom_loc.y()), 2) / (2*pow(FLAGS_sigma_y, 2)); 
        double log_prob_motion = log_prob_x + log_prob_y + log_prob_theta;

        // check low-res map
        double high_res_log_prob_map = 0;
        for (const Eigen::Vector2d& r_point : rotated_points) 
        {
          double map_x = x + r_point.x();
          double map_y = y + r_point.y();
          int lookup_x = map_x / FLAGS_raster_high_resolution;
          int lookup_y = map_y / FLAGS_raster_high_resolution;
          if (high_res_raster_map.find(std::make_pair(lookup_x, lookup_y)) != high_res_raster_map.end()) {
            high_res_log_prob_map += high_res_raster_map[std::make_pair(lookup_x, lookup_y)];
          }
        }

        double log_prob = log_prob_motion_constant + log_prob_motion + high_res_log_prob_map;
        if (log_prob > bestLogProb) {
          bestLogProb = log_prob;
          bestIdx = idx;
          bestPose = Eigen::Vector3d(x, y, angle);
        }
      }
    }
  }
  // add best cov to what we send back
    double bestCov = CovPoses(bestIdx, bestIdx);
    Eigen::Vector4d bestPoseWithVar = Eigen::Vector4d(bestPose.x(), bestPose.y(), bestPose.z(), bestCov);
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
    // const Vector2d laser_location(0.2, 0);
    // Convert range and angle to Cartesian coordinates
    for (size_t i = 0; i < num_ranges; i++)
    {
      double range = ranges[i];
      if (FLAGS_slam_min_range < range && range < FLAGS_slam_max_range && range_min < range && range < range_max) {
        double angle = angle_min + i * angle_increment;
        Eigen::Vector2d point;
        point = Eigen::Vector2d(range*cos(angle), range*sin(angle));
        // point = range *  Vector2d(cos(angle), sin(angle)) + laser_location;
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
      high_res_raster_maps_.push_back(BuildHighResRasterMapFromPoints(point_cloud));
      low_res_raster_maps_.push_back(BuildLowResRasterMapFromHighRes(high_res_raster_maps_[0]));
      alignedPointsOverPoses_.push_back(point_cloud);
      optimizedPosesVariances_.push_back(0.0d);
      ready_to_csm_ = false; 
      return;
    }

    if (!ready_to_csm_) {
      return;
    }

    // Get the optimized pose
    printf("Running CSM...\n");
    Eigen::Vector4d optimized_pose_and_var = CorrelativeScanMatching(point_cloud, prev_odom_loc_, prev_odom_angle_);
    Eigen::Vector3d optimized_pose = Eigen::Vector3d(optimized_pose_and_var.x(), optimized_pose_and_var.y(), optimized_pose_and_var.z());
    double optimized_pose_variance = optimized_pose_and_var.w();
    Eigen::Vector2d optimized_loc = Eigen::Vector2d(optimized_pose.x(), optimized_pose.y());  
    double optimized_angle = optimized_pose.z();
    // update point cloud
    printf("Aligning PC...\n");
    std::vector<Eigen::Vector2d> aligned_point_cloud = AlignedPointCloud(point_cloud, optimized_loc, optimized_angle);
    std::map<std::pair<int,int>, double> high_res_raster_map = BuildHighResRasterMapFromPoints(aligned_point_cloud);
    std::map<std::pair<int,int>, double> low_res_raster_map = BuildLowResRasterMapFromHighRes(high_res_raster_map);

    // if we have too many poses, remove the oldest one
    if (num_scans >= FLAGS_slam_num_poses) {
      alignedPointsOverPoses_.erase(alignedPointsOverPoses_.begin());
      optimizedPoses_.erase(optimizedPoses_.begin());
      high_res_raster_maps_.erase(high_res_raster_maps_.begin());
      low_res_raster_maps_.erase(low_res_raster_maps_.begin());
      optimizedPosesVariances_.erase(optimizedPosesVariances_.begin());
    }

    alignedPointsOverPoses_.push_back(aligned_point_cloud);
    optimizedPoses_.push_back(optimized_pose);
    high_res_raster_maps_.push_back(high_res_raster_map);
    low_res_raster_maps_.push_back(low_res_raster_map);
    optimizedPosesVariances_.push_back(optimized_pose_variance);
    ready_to_csm_ = false; 
  }

void SLAM::ObserveOdometry(const Vector2d& odom_loc, const double odom_angle) {
  if (!odom_initialized_) {
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;    
    odom_initialized_ = true;
    ready_to_csm_ = true;
    return;
  }

  double delta_odom_x = odom_loc.x() - prev_odom_loc_.x();
  double delta_odom_y = odom_loc.y() - prev_odom_loc_.y();
  double delta_odom_angle = RadToDeg(odom_angle - prev_odom_angle_);
  double delta_odom_dist = sqrt(pow(delta_odom_x, 2) + pow(delta_odom_y, 2));
  if (delta_odom_dist < FLAGS_slam_dist_threshold && fabs(delta_odom_angle) < FLAGS_slam_angle_threshold) {
    return;
  }

  // otherwise we have deviated enough
  printf("Deviation: %f %f\n", delta_odom_dist, delta_odom_angle);
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
  printf("Num scans: %d\n", num_scans);
  Eigen::Vector3d first_pose = optimizedPoses_[0];
  Eigen::Vector2d first_loc = Eigen::Vector2d(first_pose.x(), first_pose.y());
  // double first_angle = first_pose.z();

  for (int i = 0; i < num_scans; i++) {

    Eigen::Vector3d pose = optimizedPoses_[i];
    Eigen::Vector2d loc = Eigen::Vector2d(pose.x(), pose.y());
    // double angle = pose.z();

    double delta_x = loc.x() - first_loc.x();
    double delta_y = loc.y() - first_loc.y();
    // double delta_angle = angle - first_angle;
    // double delta_angle = 30.0 * i; // ground truth
    double delta_angle = 30.0 * i * M_PI / 180.0;


    // Eigen::Rotation2Dd r_optimized_pose_to_first_pose(-1.0 * delta_angle);
    Eigen::Rotation2Dd r_optimized_pose_to_first_pose(delta_angle);
    Eigen::Matrix2d R = r_optimized_pose_to_first_pose.toRotationMatrix();
    Eigen::Matrix3d T_optimized_pose_to_first_pose;

    // float translationX = delta_x;
    // float translationY = delta_y;

    float translationX = 0.0f;
    float translationY = 0.0f;

    T_optimized_pose_to_first_pose << R(0, 0), R(0, 1), 0,
        R(1, 0), R(1, 1), 0,
        0, 0, 1;

    Eigen::Vector3d translation(translationX, translationY, 0.0f);
    printf("Delta x: %f, Delta y: %f, Delta angle: %f\n", delta_x, delta_y, delta_angle);
    vector<Eigen::Vector2d> aligned_point_cloud = alignedPointsOverPoses_[i];
    for (const Eigen::Vector2d& point : aligned_point_cloud) {
      double x = point.x();
      double y = point.y();
      // double rotated_x = x * cos(delta_angle) - y * sin(delta_angle);
      // double rotated_y = x * sin(delta_angle) + y * cos(delta_angle);

      Eigen::Vector3d point_to_transform(x, y, 1.0d);

      point_to_transform = point_to_transform - translation;
      Eigen::Vector3d transformed_point = T_optimized_pose_to_first_pose * point_to_transform;
      float transformed_x = transformed_point.x();
      float transformed_y = transformed_point.y();
      Eigen::Vector2f transformed_point_2d = Eigen::Vector2f(transformed_x, transformed_y);
      map.push_back(transformed_point_2d);



      // map.push_back(Vector2f(rotated_x + delta_x, rotated_y + delta_y));
      // map.push_back(Vector2f(x, y));
    }

    // vector<Eigen::Vector2d> aligned_point_cloud = alignedPointsOverPoses_[i];
    // for (const Eigen::Vector2d& point : aligned_point_cloud) {
    //   double x = point.x();
    //   double y = point.y();
    //   double rotated_x = x * cos(delta_angle) - y * sin(delta_angle);
    //   double rotated_y = x * sin(delta_angle) + y * cos(delta_angle);
    //   map.push_back(Vector2f(rotated_x + delta_x, rotated_y + delta_y));
    // }
  }
  return map;
}
}  // namespace slam
