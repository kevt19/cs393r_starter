#include "particle_filter.cuh"

__global__ 
void create_ray_line_segments(const float laser_loc_x, const float laser_loc_y, const float range_min, const float angle, const float angle_increment, float *ray_line_segments){
	int i = threadIdx.x;
	
	float ray_line = laser_loc_x + laser_loc_y + range_min + angle + angle_increment;
	ray_line_segments[i] = ray_line;
}
