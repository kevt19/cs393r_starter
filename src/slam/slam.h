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
\file    slam.h
\brief   SLAM Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include <map>
#include "vector_map/vector_map.h"
#include "ros/ros.h"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>

using namespace gtsam;


using vector_map::VectorMap;
using ros::Time;

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_

namespace slam {

class SLAM {
 public:
  // Default Constructor.
  explicit SLAM();

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max);

  // Observe new odometry-reported location.
  void ObserveOdometry(const Eigen::Vector2d& odom_loc,
                       const double odom_angle);

  // Get latest map.
  std::vector<Eigen::Vector2f> GetMap();
  std::map<std::pair<int,int>, double> BuildLowResRasterMapFromHighRes(std::map<std::pair<int,int>, double> high_res_raster_map); 
  std::map<std::pair<int,int>, double> BuildHighResRasterMapFromPoints(const std::vector<Eigen::Vector2d> &alignedPoints);
  std::map<std::pair<int,int>, double> BuildHighResRasterMapFromMap(const VectorMap& map);
  std::vector<Eigen::Vector2d> AlignPointCloud(const std::vector<Eigen::Vector2d>& point_cloud,
                                  const Eigen::Vector2d& optimized_loc,
                                  const double optimized_angle);
  std::vector<std::pair<Eigen::Vector3d, Eigen::MatrixXd>> CorrelativeScanMatching(const std::vector<Eigen::Vector2d>& point_cloud,
                                   const Eigen::Vector2d& odom_loc, 
                                   const double odom_angle);
  std::pair<Eigen::Vector3d, Eigen::MatrixXd> SingleCorrelativeScanMatching(const std::vector<Eigen::Vector2d>& point_cloud,
                                   const Eigen::Vector2d& odom_loc, 
                                   const double odom_angle,
                                   const Eigen::Vector2d& prev_loc,
                                   const double prev_angle,
                                   std::map<std::pair<int,int>, double> low_res_raster_map,
                                   std::map<std::pair<int,int>, double> high_res_raster_map);
  void UpdateLocation(const Eigen::Vector2d& odom_loc, const double odom_angle);
  void ObserveLocalization(const Eigen::Vector2f& odom_loc, const float odom_angle);
  void PoseGraphOptimization();
  void SetMapPointCloud();
  void ClearPreviousData();

  // Get latest robot pose.
  void GetPose(Eigen::Vector2d* loc, double* angle) const;
  bool usingGroundTruthLocalization_;


 private:
  // Previous odometry-reported locations.
  Eigen::Vector2d prev_loc_;
  double prev_angle_;
  Eigen::Vector2d current_loc_;
  double current_angle_;
  Eigen::Vector2d initial_loc_;
  double initial_angle_;
  bool ready_to_csm_;
  bool location_initialized_;
  std::vector<std::map<std::pair<int,int>, double>> high_res_raster_maps_;
  std::vector<std::map<std::pair<int,int>, double>> low_res_raster_maps_;
  std::vector<std::vector<Eigen::Vector2d>> alignedPointsOverPoses_;
  std::vector<std::vector<Eigen::Vector2d>> pointClouds_;
  std::vector<Eigen::Vector3d> optimizedPoses_;
  std::vector<Eigen::Vector3d> odometryPoses_;
  std::vector<Eigen::MatrixXd> optimizedPosesVariances_;
  int nNodesInGraph = 1;
  NonlinearFactorGraph graph_;
  Values initialGuesses_;
  std::vector<Eigen::Vector2f> map_;
  Eigen::Vector3d latestOptimizedPose_;
  std::vector<int> nodeIndices_; // node indices for each pose
  double lastTimeSLAMRan_;

  // constants
  double raster_map_gaussian_sigma_constant;
  double log_prob_x_constant;
  double log_prob_y_constant;
  double log_prob_theta_constant;
  double log_prob_motion_constant;

};
}  // namespace slam

#endif   // SRC_SLAM_H_
