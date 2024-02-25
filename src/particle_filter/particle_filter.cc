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
\file    particle-filter.cc
\brief   Particle Filter Starter Code
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
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

DEFINE_double(num_particles, 50, "Number of particles");

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // // Test variables for robot pose.  I'm also assuming the pose is w.r.t the laser frame NOT base_link.
  // Vector2f locTest(-32, 21);
  // float angleTest = 1.5;

  // Note: The returned values must be set using the `scan` variable:
  vector<Vector2f>& scan = *scan_ptr;
  scan.resize(num_ranges);

  ////// Create ray line segments
  const float angle_increment = (abs(angle_max) + abs(angle_min)) / num_ranges;
  vector<line2f> ray_line_segments;
  float theta = angle_min;    
  for (size_t i = 0; i < scan.size(); ++i) 
  {
    float px1 = range_min * cos(theta);
    float py1 = range_min * sin(theta);
    float px2 = range_max * cos(theta);
    float py2 = range_max * sin(theta);
    // Ray line endpoint coordinates with respect to vector map frame
    float x1 = px1*cos(angle) - py1*sin(angle) + loc.x(); 
    float y1 = px1*sin(angle) + py1*cos(angle) + loc.y();
    float x2 = px2*cos(angle) - py2*sin(angle) + loc.x();
    float y2 = px2*sin(angle) + py2*cos(angle) + loc.y();
    // Ray line for ith ray
    line2f ray_line(x1, y1, x2, y2);
    ray_line_segments.push_back(ray_line);
    theta += angle_increment;
  }

  ////// Obtains the closest intersection point for each ray line
  for (size_t i = 0; i < ray_line_segments.size(); i++)
  {
    line2f ray_line = ray_line_segments[i];
    vector<Vector2f> intersection_list; // Initialize a vector of intersection points
    //// Iterate through every "map line" in "vector map"
    for (size_t j = 0; j < map_.lines.size(); ++j) 
    {
      const line2f map_line = map_.lines[j];
      // Creates a vector containing all the intersection points
      Vector2f intersection_point;
      if (map_line.Intersection(ray_line, &intersection_point)) 
      {
        intersection_list.push_back(intersection_point);
      }
    }

    //// Finds the intersection point closest to the laser frame and adds it to "scan"
    scan[i] = Vector2f(0, 0);
    float smallest = std::numeric_limits<float>::max();
    for (Vector2f point: intersection_list)
    {
      float point_distance = (point - loc).norm();
      if (point_distance < smallest)
      {
        smallest = point_distance;
        scan[i] = point;
      }
    }
  }
  // ////// Prints every point in "scan"
  // for (size_t k = 0; k < scan.size(); k++)
  // {
  //   printf("Point %ld: [%f, %f]\n", k, scan[k].x(), scan[k].y());
  // }
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.
}

void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable. 
  // Create a variable to store the new particles, and when done, replace the
  // old set of particles:
  // vector<Particle> new_particles';
  // During resampling: 
  //    new_particles.push_back(...)
  // After resampling:
  // particles_ = new_particles;

  // You will need to use the uniform random number generator provided. For
  // example, to generate a random number between 0 and 1:
  float x = rng_.UniformRandom(0, 1);
  printf("Random number drawn from uniform distribution between 0 and 1: %f\n",
         x);
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.
}

void ParticleFilter::Predict(const Vector2f& odom_loc,
                             const float odom_angle) {
  // Implement the predict step of the particle filter here.
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.


  // You will need to use the Gaussian random number generator provided. For
  // example, to generate a random number from a Gaussian with mean 0, and
  // standard deviation 2:
  float x = rng_.Gaussian(0.0, 2.0);
  printf("Random number drawn from Gaussian distribution with 0 mean and "
         "standard deviation of 2 : %f\n", x);
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load(map_file);
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:
  loc = Vector2f(0, 0);
  angle = 0;
}


}  // namespace particle_filter
