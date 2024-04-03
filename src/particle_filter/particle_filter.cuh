#ifndef SRC_PARTICLE_FILTER_CUH_
#define SRC_PARTICLE_FILTER_CUH_

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <cuda.h>
// #include "eigen3/Eigen/Dense"

// #include "shared/math/geometry.h"


__global__ void create_ray_line_segments(const float laser_loc_x, const float laser_loc_y, const float range_min, const float angle, const float angle_increment, float *ray_line_segments);


#endif
