#include "particle_filter.cuh"

__global__ 
void create_ray_line_segments(const Eigen::Vector2f laser_loc, const float range_min, const float angle, const float angle_increment, geometry::line2f *ray_line_segments){
	int i = threadIdx.x;
	
	line2f ray_line(laser_loc[0], range_min, angle, angle_increment);
	ray_line_segments[i] = ray_line;
}
