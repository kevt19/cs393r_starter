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

// #include <gtsam/geometry/Pose2.h>
// #include <gtsam/nonlinear/NonlinearFactorGraph.h>
// #include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
// #include <gtsam/nonlinear/Values.h>
// #include <gtsam/slam/BetweenFactor.h>
// #include <gtsam/inference/Symbol.h>
// #include <gtsam/slam/PriorFactor.h>

// using namespace gtsam;
using vector_map::VectorMap;

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
  void BuildRasterMapsFromPoints(const std::vector<Eigen::Vector2d> &points);
  void BuildRasterMapsFromMap(const VectorMap& map);
  std::vector<Eigen::Vector2d> AlignedPointCloud(const std::vector<Eigen::Vector2d>& point_cloud,
                                  const Eigen::Vector2d& optimized_loc,
                                  const double optimized_angle);
  Eigen::Vector3d CorrelativeScanMatching(const std::vector<Eigen::Vector2d>& point_cloud,
                                   const Eigen::Vector2d& odom_loc, 
                                   const double odom_angle);
  Eigen::Vector3d SingleCorrelativeScanMatching(const std::vector<Eigen::Vector2d>& point_cloud,
                                   const Eigen::Vector2d& odom_loc, 
                                   const double odom_angle,
                                   const Eigen::Vector2d& prev_loc,
                                   const double prev_angle);

  // Get latest robot pose.
  void GetPose(Eigen::Vector2d* loc, double* angle) const;

 private:
  // Previous odometry-reported locations.
  Eigen::Vector2d prev_odom_loc_;
  bool readyToSlam_;
  double prev_odom_angle_;
  bool odom_initialized_;
  std::map<std::pair<int,int>, double> high_res_raster_map_;
  std::map<std::pair<int,int>, double> low_res_raster_map_;
  std::vector<std::vector<Eigen::Vector2d>> alignedPointsOverPoses_;
  std::vector<Eigen::Vector3d> optimizedPoses_;
};
}  // namespace slam

#endif   // SRC_SLAM_H_
