#ifndef SRC_PARTICLE_FILTER_CUH_
#define SRC_PARTICLE_FILTER_CUH_

#include <cuda_runtime.h>
#include "eigen3/Eigen/Dense"

#include "shared/math/geometry.h"


__global__ void create_ray_line_segments(const Eigen::Vector2f laser_loc, const float range_min, const float angle, const float angle_increment, geometry::line2f *ray_line_segments);


#endif
