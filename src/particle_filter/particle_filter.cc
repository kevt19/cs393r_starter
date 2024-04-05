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

#include <queue>

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;
using math_util::AngleDiff;

/*
.steps to tune (lecture 08 slide 42):
1. start with simple observation likelihood function
2. test in simulation so there are no unexpected observations
3. tune std dev and gamma to prevent overconfident estimates
4. then implement robust piece-wise observation likelihood function
5. use logged data to tune parameters

. common issues (lecture 08 slide 43):
1. slow run time
  use fewer rays in scan
  use fewer particles
  look for places to reuse computation

2. particle cloud converges too quickly without much information
  observation likelihood is overconfident; increase std dev, decrease gamma

3. particle cloud diverges from true car location
  a. is there sufficient overlap with the proposal distribution and true location?
    indicates undersampling from the motion model, should increase the varaince multilier constants and/or number of particles
  b. are all the particle weights comparable?
    use low-variance resampling and/or resample less often
  c. are the wrong sets of particles being assigned higher weights?
    parameters of the robust observation likelihood function are probably counter-productive
  d. are the weights infinitesimally small?
    might be reaching limit of numerical precision, indicates obervation likelihod function is overconfident
*/

// . efficiency~accuracy tradeoff parameters
#define n_particles 35 // number of particles instantiated and resampled by filter
#define laser_downsampling_factor 10 // downsampled laser scan will be 1/N its original size; used to improve computational efficiency
#define resampling_iteration_threshold 15// resampling only occurs every n iterations of particle weight updates

// . observation model parameters
#define d_long 1.0d // 
#define d_short 0.5d // 
#define sigma_s 0.75d // std deviation of the LiDAR sensor measurements
#define gamma 0.75d // scalar on the weight updates for each point in the scan

// . motion model noise parameters
DEFINE_double(k1, 0.5, "Error in translation from translation motion");
DEFINE_double(k2, 0.5, "Error in rotation from translation motion");
DEFINE_double(k3, 0.1, "Error in rotation from rotation motion");
DEFINE_double(k4, 0.1, "Error in translation from rotation motion");
DEFINE_double(k5, 0.5, "Error in translation from translation motion along major axis");
DEFINE_double(k6, 0.5, "Error in translation from translation motion along minor axis");

// . fixed
DEFINE_double(pi, 3.1415926, "Pi");

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false),
    resampling_iteration_counter_(0) {}

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
  // The returned values must be set using the 'scan' variable:
  scan.resize(num_ranges / laser_downsampling_factor);

  // Calculate lidar location (0.2m in front of base_link)
  Eigen::Vector2f lidar_loc = loc + 0.2 * Vector2f(cos(angle), sin(angle));

  // Loop through laser scans creating a line for each ray
  float angle_increment = (angle_max - angle_min) / num_ranges * laser_downsampling_factor;
  for (size_t i = 0; i < scan.size(); i++) {
    // Calculate angle of the ray
    float ray_angle = angle + angle_min + i * angle_increment;

    // Create a line for the ray
    line2f ray(
      lidar_loc.x() + range_min * cos(ray_angle),
      lidar_loc.y() + range_min * sin(ray_angle),
      lidar_loc.x() + range_max * cos(ray_angle),
      lidar_loc.y() + range_max * sin(ray_angle)
    );

    // Laserscan maximum value is default ray intersection with the map
    Eigen::Vector2f ray_intersection = lidar_loc + range_max * Vector2f(cos(ray_angle), sin(ray_angle));
    float obstacle_dist = range_max;

    // Loop through map lines checking for intersections with the predicted ray
    for (size_t j = 0; j < map_.lines.size(); ++j) {
      const line2f map_line = map_.lines[j];

      // Check for intersection between ray and map line
      Eigen::Vector2f intersection_point;
      bool intersects = map_line.Intersection(ray, &intersection_point);

      // Update if the intersection is closer (deal with multiple collisions, take first obstacle seen)
      if (intersects && (intersection_point - lidar_loc).norm() < obstacle_dist) {
        obstacle_dist = (intersection_point - lidar_loc).norm();    // Update distance
        ray_intersection = intersection_point;    // Update location
      }
    }

    // Lastly, add obstacle to generated scan
    scan[i] = ray_intersection;
  }
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Lecture 08 slide 44, Lecture 7 slide 32, log likelihoods and infitesimally small numbers

  // getting the expected cloud at the particle pose
  std::vector<Eigen::Vector2f> predicted_scan; // This scan will be altered by GetPredictedPointCloud to be compared to ranges
  Particle particle = *p_ptr;
  GetPredictedPointCloud(particle.loc, particle.angle, ranges.size(), range_min, range_max, angle_min, angle_max, &predicted_scan);

  // downsampling the raw scan to make its size match the predicted scan
  int downsampling_factor {static_cast<int>(ranges.size() / predicted_scan.size())};
  std::vector<float> downsampled_ranges(predicted_scan.size());
  for (std::size_t i = 0; i < predicted_scan.size(); i++) {
    downsampled_ranges[i] = ranges[i * downsampling_factor];
  }

  // the particle's weight
  double default_particle_weight {1e-06d}; // setting initial weight in probability space to be small value (nonzero so log is not indeterminant)
  double log_weight {log(default_particle_weight)}; // working with weights in log space because we'll want them there for resampling anyway

  // getting location of LiDAR sensor given the particle's pose
  const Eigen::Vector2f kLaserLoc(0.2, 0); // pulled from navigation_main.cc
  Eigen::Vector2f lidar_location = particle.loc + kLaserLoc(0) * Eigen::Vector2f(cos(particle.angle), sin(particle.angle));

  // iterating over the points in the predicted scan
  for (std::size_t i = 0; i < predicted_scan.size(); i++) {
    
    // getting the range (magnitude of distance) between the point and the LiDAR sensor's location
    double predicted_scan_range {(predicted_scan[i] - lidar_location).norm()}; 

    // // . this is the simple observation likelihood function
    // log_weight += (-1 * pow((downsampled_ranges[i] - predicted_scan_range), 2) / (pow(sigma_s, 2)));

    // . this is a more complete observation likelihood function
    // ranges less than the minimum range or greater than the maximum range are excluded; adding some tolerance so 9.999 and 0.201 don't ruin us
    // if (downsampled_ranges[i] < range_min || downsampled_ranges[i] > range_max){
    float tolerance {0.05};
    if (downsampled_ranges[i] < (1 + tolerance) * range_min || downsampled_ranges[i] > (1 - tolerance) * range_max) {continue;}

    // if the predicted scan range is less than some value (pred - d_s), treat it as uniform instead of Gaussian
    if (downsampled_ranges[i] < predicted_scan_range - d_short) {
      log_weight += (-1 * pow(d_short, 2) / pow(sigma_s, 2));
    }
    // if the predicted scan range is greater than some value (pred + d_l), treat it as uniform instead of Gaussian
    else if (downsampled_ranges[i] > predicted_scan_range+d_long) {
      log_weight += (-1 * pow(d_long, 2) / pow(sigma_s, 2));
    }
    // simplest case, assumes purely Gaussian distribution between expected and actual ranges
    else {
      log_weight += (-1 * pow((downsampled_ranges[i] - predicted_scan_range), 2) / (pow(sigma_s, 2)));
    }
  }
  // assigning weight with some scalar gamma
  p_ptr->weight = gamma * log_weight;
}

void ParticleFilter::Resample() {
  // checking to see if we have anything to resample
  if (particles_.empty()) {
      std::cerr << "NO EXISTING PARTICLES! Nothing to resample..." << std::endl;
      return;
  }

  // create vector for new particles
  std::vector<particle_filter::Particle> resampled_particles;
  resampled_particles.reserve(n_particles);

  // build discrete distribution for resampling
  double weights_sum {0.d};
  std::queue<double> relative_positions;
  for (std::size_t i = 0; i < particles_.size(); i++) {
    weights_sum += exp(particles_[i].weight); // converting this out of log space to get the range for low-variance resampling
    relative_positions.push(weights_sum); // keeping track of where the particles fall in the distribution we are going to resample for convenince
  }

  // create starting point and step size for low-variance resampling
  double low_variance_sampling_step {weights_sum / n_particles}; // size of constant step for low variance resampling
  double current_sampling_location {rng_.UniformRandom(0, low_variance_sampling_step)}; // starting point for low-variance resampling; constraining it to be within the first step so we don't have to worry about wrapping around

  // resampling
  int original_particle_counter {0}; // to track which particle from the original vector to add to the resampled vector
  // iterate for the total number of particles
  for (std::size_t i = 0; i < n_particles; i++) {
    while (!relative_positions.empty()) {
      if (relative_positions.front() < current_sampling_location) { // checking to see if the front of the discrete probability distribution queue is less than the current sampling location; if it is, we need to move to the next bin to sample
        relative_positions.pop();
        original_particle_counter++;
      } else { // otherwise, we are in the correct bin and should draw a particle for the resampled distribution
        resampled_particles.push_back(particles_[original_particle_counter]);
        break; // to make sure we only add once particle for each iteration
      }
    }
    current_sampling_location += low_variance_sampling_step;
  }

  // make sure we resampled how many particles we wanted
  if (static_cast<int>(resampled_particles.size()) != n_particles) {
    std::cerr << resampled_particles.size() << " particles were resampled instead of " << n_particles << "! Investigate this." << std::endl;
  }

  // reset particle weights after resampling
  double resampled_weight {log(1.d / n_particles)};
  for (auto &particle : resampled_particles) {
    particle.weight = resampled_weight;
  }

  // assign the resampled particles
  particles_ = resampled_particles;
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // Ignore callback when odometry has not changed (car has not moved more than some threshold; refer to Predict() function for more information)
  if (!new_odometry_flag_) {
    return;
  }

  // update the weights of each particle
  Particle *top_particle = new Particle;
  double max_particle_weight {-100000};
  for (auto &particle : particles_) {
    Update(ranges, range_min, range_max, angle_min, angle_max, &particle);
    if (particle.weight > max_particle_weight) {
      max_particle_weight = particle.weight;
      top_particle = &particle;
    }
  }
  for (auto &particle : particles_) {
    particle.weight -= max_particle_weight;
  }
  *top_particle_ = *top_particle;   // Only copy the contents to avoid changes in between particle

  // check how many iterations we have had since resampling, resample if it's time to do so
  resampling_iteration_counter_++;
  if (resampling_iteration_counter_ == resampling_iteration_threshold) {
    resampling_iteration_counter_ = 0;
    Resample();
  }
}

// A new odometry value is available. Propagate the particles forward using the motion model.
void ParticleFilter::Predict(const Vector2f& odom_loc,
                             const float odom_angle) {
  // Ignore new pose set
  if (odom_initialized_) {
    // Calculate pose change from odometry reading
    Eigen::Vector2f translation_diff = odom_loc - prev_odom_loc_;
    float rotation_diff = odom_angle - prev_odom_angle_;

    // Set flag if odometry has change
    if (translation_diff.norm() < 0.005 && rotation_diff < 0.005) {
      new_odometry_flag_ = false;
      return;
    }
    else {
      new_odometry_flag_ = true;
    }

    // Ignore unrealistic jumps in odometry
    if (translation_diff.norm() < 1.0 && abs(rotation_diff) < FLAGS_pi / 4) {
      // Loop through particles
      for (auto &particle : particles_) {
        // Transform odometry pose change to map frame (for particle)
        Eigen::Rotation2Df rotation_vector(AngleDiff(particle.angle, prev_odom_angle_));
        Eigen::Vector2f particle_translation = rotation_vector * translation_diff;

        // // Sample noise from a Gaussian distribution
        // float x_noise = rng_.Gaussian(0.0,  FLAGS_k1 * translation_diff.norm() + FLAGS_k4 * rotation_diff);
        // float y_noise = rng_.Gaussian(0.0,  FLAGS_k1 * translation_diff.norm() + FLAGS_k4 * rotation_diff);
        // float rotation_noise = rng_.Gaussian(0.0, FLAGS_k2 * translation_diff.norm() + FLAGS_k3 * rotation_diff);

        // Sample noise from a Gaussian distribution for a more complete ellipsoid model
        auto translation_norm {translation_diff.norm()};
        auto major_axis_noise {rng_.Gaussian(0.0, FLAGS_k5 * translation_norm + FLAGS_k4 * rotation_diff)};
        auto minor_axis_noise {rng_.Gaussian(0.0, FLAGS_k6 * translation_norm + FLAGS_k4 * rotation_diff)};

        auto x_noise {major_axis_noise * cos(particle.angle) + minor_axis_noise * sin(particle.angle)};
        auto y_noise {major_axis_noise * sin(particle.angle) + minor_axis_noise * cos(particle.angle)};
        auto rotation_noise {rng_.Gaussian(0.0, FLAGS_k2 * translation_diff.norm() + FLAGS_k3 * rotation_diff)};

        // Update particle location from motion model
        particle.loc[0] += particle_translation[0] + x_noise;
        particle.loc[1] += particle_translation[1] + y_noise;
        particle.angle += rotation_diff + rotation_noise;
      }
    }
  }
  else {
    prev_odom_loc_ = odom_loc;
    prev_odom_angle_ = odom_angle;
    odom_initialized_ = true;
  }

  // Update previous odometry
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  map_.Load(map_file);

  // Initialize odometry
  prev_odom_loc_ = loc;
  prev_odom_angle_ = angle;
  odom_initialized_ = false;

  // Clear and Initialize particles
  particles_.clear();
  for (int i = 0; i < n_particles; i++) {
    // Generate random number for error
    float error_x = rng_.UniformRandom(-0.25, 0.25);  
    float error_y = rng_.UniformRandom(-0.25, 0.25);
    float error_theta = rng_.UniformRandom(-FLAGS_pi / 4, FLAGS_pi / 4);

    // Create particle using error weights
    Particle p = {
      Eigen::Vector2f(loc[0] + error_x, loc[1] + error_y),
      (float)(angle + error_theta),
      1 / n_particles};

    // Add particle
    particles_.push_back(p);
  }

  std::cout << particles_.size() << " particles initialized" << std::endl;
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;

  // - for just returning the most likely particle
  // loc = top_particle_->loc;
  // angle = top_particle_->angle;

  // - taking weighted average around most likely particle
  double radial_inclusion_distance {5.0}; // m
  auto most_likely_location {top_particle_->loc};

  auto location_estimate {top_particle_->loc};
  auto angle_estimate {top_particle_->angle};
  int total_considered_particles {1};
  auto sum_of_weights {exp(top_particle_->weight)};

  double radial_inclusion_distance_sqrd {pow(radial_inclusion_distance, 2)};
  for (std::size_t i = 0; i < particles_.size(); i++) {
    
    // calculate the distance from the most likely location to the particle
    double radial_distance_sqrd {pow(particles_[i].loc.x() - most_likely_location.x(), 2) + pow(particles_[i].loc.y() - most_likely_location.y(), 2)};

    // don't consider points farther from the most likely particle than the set distance
    if (radial_distance_sqrd > radial_inclusion_distance_sqrd) {continue;}

    // accumulate the point for calculations (probably start with a simple average, then maybe can consider a probability- or distance-weighted average depending on the results)
    total_considered_particles++;
    auto weight {exp(particles_[i].weight)};
    location_estimate.x() += weight * particles_[i].loc.x();
    location_estimate.y() += weight * particles_[i].loc.y();
    angle_estimate += weight * particles_[i].angle;
    sum_of_weights += weight;
  }

  // divide to get averages
  location_estimate.x() /= sum_of_weights;
  location_estimate.y() /= sum_of_weights;
  angle_estimate /= sum_of_weights;

  loc = location_estimate;
  angle = angle_estimate;

  // std::cout << "Estimated location: " << loc.transpose() << std::endl;
}

}  // namespace particle_filter
