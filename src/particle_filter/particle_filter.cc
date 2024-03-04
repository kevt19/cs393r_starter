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

DEFINE_double(stddev, 0.05, "LiDAR Sensor Noise, 0.05m - 1m");
DEFINE_double(gamma, 0.1, "Ray correlation [1/num_particles,1]");
DEFINE_double(d_long, 1, "Used in Observation Likelihood Model, has to be higher than d_short, 1m - 1.5m");
DEFINE_double(d_short, 0.2, "Used in Observation Likelihood Model, around .2m - .5m");

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
                                            int num_ranges_full,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr,
                                            vector<double>* ranges_ptr) {

  float num_ranges = num_ranges_full/47; // Creates 23 points out of 1081
  vector<Vector2f>& scan = *scan_ptr;
  scan.resize(num_ranges);

  vector<double>& ranges = *ranges_ptr;
  ranges.resize(num_ranges);

  Vector2f laser_loc = loc + 0.2*Vector2f(cos(angle), sin(angle));

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
    float x1 = px1*cos(angle) - py1*sin(angle) + laser_loc.x(); 
    float y1 = px1*sin(angle) + py1*cos(angle) + laser_loc.y();
    float x2 = px2*cos(angle) - py2*sin(angle) + laser_loc.x();
    float y2 = px2*sin(angle) + py2*cos(angle) + laser_loc.y();
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
    scan[i] = Vector2f(range_max, 0);
    float smallest = range_max;
    ranges[i] = smallest;
    for (Vector2f point: intersection_list)
    {
      float point_distance = (point - laser_loc).norm();
      if (point_distance < smallest)
      {
        smallest = point_distance;
        scan[i] = point;

        ranges[i] = smallest;
      }
    }
  }
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  ////// Initialize the predicted scan from GetPredictedPointCloud()
  vector<Vector2f> predicted_scan;
  vector<double> predicted_ranges;
  GetPredictedPointCloud(p_ptr->loc, p_ptr->angle, ranges.size(), range_min, range_max, angle_min, angle_max, &predicted_scan, &predicted_ranges);
  double log_likelihood = 0;
  for (size_t i = 0; i < ranges.size(); i += 47)
  {
    if (ranges[i] < range_min || ranges[i] > range_max){
      log_likelihood += 0;
    }

    else if (ranges[i] < predicted_ranges[i] - FLAGS_d_short){
      log_likelihood += -(pow(FLAGS_d_short,2)/pow(FLAGS_stddev,2));
    }

    else if (ranges[i] > predicted_ranges[i] + FLAGS_d_long){
      log_likelihood += -(pow(FLAGS_d_long,2))/(pow(FLAGS_stddev,2));
    }

    else{
      log_likelihood += FLAGS_gamma * -(pow((ranges[i] - predicted_ranges[i]),2))/(pow(FLAGS_stddev,2));
    }

    // log_likelihood +=  FLAGS_gamma * (-0.5) * (pow(ranges[i] - predicted_ranges[i], 2) / pow(FLAGS_stddev, 2));  // Simple Version
  }
  p_ptr->weight = 1 / abs(log_likelihood);
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

  vector<Particle> new_particles;

  double sum_of_weights = 0;
  for(auto particle : particles_){
    sum_of_weights += particle.weight;
  }

  for(size_t i = 0; i < particles_.size(); i++){
    float x = rng_.UniformRandom(0, sum_of_weights);

    float running_w = 0;
    for(auto particle : particles_){
      running_w += particle.weight;
      if(running_w > x){
        new_particles.push_back(particle);
        break;
      }
    }
  }

  particles_ = new_particles;
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.

  for(auto& particle : particles_){
    Update(ranges, range_min, range_max, angle_min, angle_max, &particle);
  }

  Resample();
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
  Particle best_particle = particles_[0];
  double best_weight = 0;
  for(auto particle : particles_){
    if(particle.weight > best_weight){
      best_particle = particle;
      best_weight = particle.weight;
    }
  }
  loc = best_particle.loc;
  angle = best_particle.angle;
}


}  // namespace particle_filter
