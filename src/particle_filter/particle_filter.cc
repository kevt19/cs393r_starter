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

using Eigen::Vector2f;
using Eigen::Vector2i;
using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;

DEFINE_double(num_particles, 50, "Number of particles");

DEFINE_double(k1, 0.3, "Weighting factor for how much translation affects translation uncertainty");
DEFINE_double(k2, 0.3, "Weighting factor for how much rotation affects translation uncertainty");
DEFINE_double(k3, 0.15, "Weighting factor for how much translation affects rotation uncertainty");
DEFINE_double(k4, 0.15, "Weighting factor for how much rotation affects rotation uncertainty");

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

  vector<Vector2f>& scan = *scan_ptr;
  scan.resize(num_ranges);

  Vector2f laser_loc = loc + 0.2*Vector2f(cos(angle), sin(angle));

  ////// Create ray line segments
  const float angle_increment = (abs(angle_max) + abs(angle_min)) / num_ranges;
  float theta = angle_min;    
  for (size_t i = 0; i < scan.size(); ++i) 
  {
    float px1 = range_min * cos(theta);
    float py1 = range_min * sin(theta);
    float px2 = range_max * cos(theta);
    float py2 = range_max * sin(theta);
    // Ray line endpoint coordinates with respect to vector map frame
    float x1 = px1*cos(angle) - py1*sin(angle) + laser_loc.x(); 
    float y1 = px1*sin(angle) + py1*cos(angle) + laser_loc.y();
    float x2 = px2*cos(angle) - py2*sin(angle) + laser_loc.x();
    float y2 = px2*sin(angle) + py2*cos(angle) + laser_loc.y();
    // Ray line for ith ray
    line2f ray_line(x1, y1, x2, y2);

    //// Iterate through every "map line" in "vector map"
    for (size_t j = 0; j < map_.lines.size(); ++j) 
    {
      Vector2f intersection_point;
      const line2f map_line = map_.lines[j];
      bool intersects = map_line.Intersects(ray_line);

      ////// Obtains the closest intersection point for each ray line
      if (intersects)
      {
        // Creates a vector containing all the intersection points
        vector<Vector2f> intersection_list; // Initialize a vector of intersection points
        if (map_line.Intersection(ray_line, &intersection_point)) 
        {
          intersection_list.push_back(intersection_point);
        }

        //// Finds the intersection point closest to the laser frame and adds it to "scan"
        scan[i] = Vector2f(0, 0);
        float smallest = std::numeric_limits<float>::max();
        for (Vector2f point: intersection_list)
        {
          float point_distance = (point - laser_loc).norm();
          if (point_distance < smallest)
          {
            smallest = point_distance;
            scan[i] = point;
          }
        }
      }
      else
      {
        scan[i] = Vector2f(x2, y2);  // If there is no collison set it to the point from the maximum range (range_max)
      }
    }

    theta += angle_increment;
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
  // Observed ranges from sensor scan
  vector<float> observed_range = ranges;
  int num_ranges = ranges.size();

  // Particle location (base_link wrt map_frame)
  Vector2f loc = p_ptr->loc;

  // Particle angle (WRT map frame)
  float angle = p_ptr->angle;

  // Particle laser location (laser_frame wrt map_frame)
  Vector2f laser_loc = loc + 0.2*Vector2f(cos(angle), sin(angle));

  // Initialize Particle Weight
  double weight = p_ptr->weight;

  // Sensor noise (To be tuned, increasing this makes it more robust, according to lecture)
  float sigma_s = 0.05; 

  // Normalization constant
  float k = 1/(sigma_s * sqrt(2*M_PI)); // This might have to be called putside this function when we sum all the particle weights

  // gamma accounts for ray correlation
  float gamma = 0.5; // [1/n, 1] ---> n=1081 ---> [.00092, 1]
  
  ////// Initialize the predicted pointcloud from GetPredictedPointCloud()
  vector<Vector2f> predicted_pointcloud;
  GetPredictedPointCloud(loc, angle, num_ranges, range_min, range_max, angle_min, angle_max, &predicted_pointcloud);
  float probabilities_sum = 0;
  for (int i = 0; i < num_ranges; i++)
  {
    float predicted_range = (laser_loc - predicted_pointcloud[i]).norm();
    float p = pow( ((-1/2) * (pow(observed_range[i] - predicted_range, 2) / pow(sigma_s, 2)) ), gamma);
    probabilities_sum += p;
  }

  weight = k * probabilities_sum;

  printf("Weight = %f\n", weight);

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

  // Initialize odometry after first observation is taken
  if(!odom_initialized_){
    prev_odom_loc_ = odom_loc;
    prev_odom_angle_ = odom_angle;
    odom_initialized_ = true;
  }

  // Convert the robot's pose in odom frame to be in map frame
  Eigen::Rotation2Df r_odom_to_baselink(-prev_odom_angle_);
  Vector2f delta_baselink = r_odom_to_baselink * (odom_loc - prev_odom_loc_);
  Eigen::Rotation2Df r_map_to_baselink(prev_map_angle_);
  Vector2f map_loc = prev_map_loc_ + r_map_to_baselink * delta_baselink;
  float delta_theta_baselink = odom_angle - prev_odom_angle_;
  float map_angle = prev_map_angle_ + delta_theta_baselink;

  // Calculate change in pose in map frame
  float delta_x = map_loc[0] - prev_map_loc_[0];
  float delta_y = map_loc[1] - prev_map_loc_[1];
  float delta_theta = map_angle - prev_map_angle_;

  // Determine uncertainty for translation motion
  float translation_uncertainty_t = FLAGS_k1 * sqrt(delta_x * delta_x + delta_y * delta_y);
  float rotation_uncertainty_t = FLAGS_k2 * abs(delta_theta);
  float std_dev_t = translation_uncertainty_t + rotation_uncertainty_t;

  // Determine uncertainty for rotational motion
  float translation_uncertainty_r = FLAGS_k3 * sqrt(delta_x * delta_x + delta_y * delta_y);
  float rotation_uncertainty_r = FLAGS_k4 * abs(delta_theta);
  float std_dev_r = translation_uncertainty_r + rotation_uncertainty_r;

  // Apply some random noise to each particle based on a simple motion model
  for (auto &particle : particles_) {
    float x_noise = rng_.Gaussian(0.0, std_dev_t);
    float y_noise = rng_.Gaussian(0.0, std_dev_t);
    float theta_noise = rng_.Gaussian(0.0, std_dev_r);

    float noisy_delta_x = delta_x + x_noise;
    float noisy_delta_y = delta_y + y_noise;
    float noisy_delta_theta = delta_theta + theta_noise;

    particle.loc[0] += noisy_delta_x;
    particle.loc[1] += noisy_delta_y;
    particle.angle += noisy_delta_theta;
  }

  // Update previous values
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;
  prev_map_loc_ = map_loc;
  prev_map_angle_ = map_angle;
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load(map_file);

  // Create num_particles random particles each starting at the car's initial position
  particles_.clear();
  for (int i = 0; i < FLAGS_num_particles; i++) {
    Particle particle;
    particle.loc = loc;
    particle.angle = angle;
    particle.weight = 0;
    particles_.push_back(particle);
  }

  prev_map_angle_ = angle;
  prev_map_loc_ = loc;
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
