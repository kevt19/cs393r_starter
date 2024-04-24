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
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "slam.h"

#include "vector_map/vector_map.h"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>

using namespace gtsam;

using namespace math_util;
using Eigen::Affine2f;
using Eigen::Rotation2Df;
using Eigen::Translation2f;
using Eigen::Vector2f;
using Eigen::Vector2i;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;

namespace slam {

SLAM::SLAM() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}
    // float max_prob_pose_value = -1000000.0;
    // int k = 0;


void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
  *loc = Vector2f(0, 0);
  *angle = 0;
}

void SLAM::ObserveLaser(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  if (!odom_initialized_) {
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;    
    odom_initialized_ = true;
    return;
  }

  // k = k+1;

  // Eigen::Vector2f loc; 
  // float angle;
  // GetPose(&loc, &angle);
  // printf("%f\n", angle);

  // float delta_odom_x = odom_loc.x() - prev_odom_loc_.x();
  // float delta_odom_y = odom_loc.y() - prev_odom_loc_.y();
  // float delta_odom_angle = odom_angle - prev_odom_angle_;

  ////// Motion Model //////////////////////
  // Parameters for creating Predicted Poses 
  double startX = odom_loc.x();
  double startY = odom_loc.y();
  double startAngle = odom_angle;

  int numStepsX = 10;
  int numStepsY = 10;
  int numStepsAngle = 15;

  double incrementX = 0.01;
  double incrementY = 0.01;
  double incrementAngle = 0.01;

  // Vector to store the Eigen vectors
  std::vector<Eigen::Vector3d> predicted_poses;

  for (int i = 0; i < numStepsX; ++i) {
      double x = startX + i * incrementX;
      for (int j = 0; j < numStepsY; ++j) {
          double y = startY + j * incrementY;
          for (int k = 0; k < numStepsAngle; ++k) {
              double angle = startAngle + k * incrementAngle;

              // Create an Eigen vector and store the current predicted pose combination
              Eigen::Vector3d vec(x, y, angle);
              predicted_poses.push_back(vec);
          }
      }
  }

  // for (Eigen::Vector3d pose: predicted_poses)
  // {
  //   printf("pose = [x = %f, y = %f, theta = %f]\n", pose[0], pose[1], pose[2]);
  // }

  float max_prob_pose_value = 1000000.0;
  Eigen::Vector3d max_prob_pose;

  // Parameters, figure out what these need to be
  float sigma_x = 0.01;
  float sigma_y = 0.01;
  float sigma_theta = 0.01;

  for (Eigen::Vector3d pose: predicted_poses)
  {
    float x = pow((pose[0] - odom_loc.x()),2) / (2*pow(sigma_x,2)); 
    float y = pow((pose[1] - odom_loc.y()),2) / (2*pow(sigma_y,2)); 
    float theta = pow((pose[2] - odom_angle),2) / (2*pow(sigma_theta,2));
    float sum = x + y + theta;
    printf("pose[0] = %f, odom_loc.x() = %f\n", pose[0], odom_loc.x());
    printf("[x = %f, y = %f, theta = %f, sum = %f]\n", x, y, theta, sum);

    if (sum < max_prob_pose_value)
    {
      printf("I made it in\n \n ");
      max_prob_pose_value = sum;
      max_prob_pose = Eigen::Vector3d(pose[0], pose[1], pose[2]);
    }
  }

  printf("Max Pose: [x = %f, y = %f, theta = %f, Probability = %f]\n", max_prob_pose[0], max_prob_pose[1], max_prob_pose[2], max_prob_pose_value);

  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;

  // Keep track of odometry to estimate how far the robot has moved between 
  // poses.
}

vector<Vector2f> SLAM::GetMap() {
  vector<Vector2f> map;
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  return map;
}
}  // namespace slam
