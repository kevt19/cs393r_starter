#include "path_options.h"
#include <cstdio>
#include <iostream>
// #include <cmath>
// 1d time optimal control
// given distance to go, max decel, max vel
// out vel

// do this on path option after selecting it

float run1DTimeOptimalControl(float dist_to_go, float current_speed, const navigation::NavigationParams& robot_config) {
    float max_accel = robot_config.max_accel;
    float max_decel = robot_config.max_decel;
    float max_vel = robot_config.max_vel;
    float dt = robot_config.dt;
    float cruise_stopping_dist = pow(current_speed, 2) / (2 * max_decel);
    float accel_stopping_dist = pow(current_speed + dt * max_accel, 2) / (2 * max_decel);
    // std::cout << "Current Speed: " << current_speed << std::endl;
    // std::cout << "Max Velocity: " << max_vel << std::endl;
    // std::cout << "dt: " << dt << std::endl;
    // std::cout << "Cruise Stopping Distance: " << cruise_stopping_dist << std::endl;
    // std::cout << "Dist to go: " << dist_to_go << std::endl;

    // if dist_to_go is larger than stopping_dist and not at max vel, can accelerate
    if (dist_to_go > accel_stopping_dist && current_speed < max_vel) {
        return std::min(max_vel, current_speed + max_accel * dt);
    }
    else if (dist_to_go > cruise_stopping_dist && current_speed == max_vel) {  // can stop in time and at max vel
                                                                        // probably needs hysteresis
        return current_speed;
    }
    else {  // otherwise needs to decelerate
        return std::max(current_speed - max_decel * dt, 0.0f);
    }
}



// set curvature, free path length, obstruction for a path option
void setPathOption(navigation::PathOption& path_option,
                        float curvature, const vector<Eigen::Vector2f>& point_cloud,
                        const navigation::NavigationParams& robot_config) {
    path_option.curvature = curvature;
    float h = robot_config.length - robot_config.base_link_offset; // distance from base link to front bumper
    if (curvature == 0) {
        for (auto p: point_cloud) {
            if (robot_config.width/2 + robot_config.safety_margin >= abs(p[1])
                && p[0] < path_option.free_path_length) {
                path_option.free_path_length = p[0] - h - robot_config.safety_margin;
                path_option.obstruction = p;
            }
        }
        // clearance
        for (auto p: point_cloud) {
            if (p[0] >=0 and p[0] < path_option.free_path_length) {
                float clearance_p = abs(p[1]) - robot_config.width / 2 - robot_config.safety_margin;
                if (clearance_p < path_option.clearance) {
                    path_option.clearance = clearance_p;
                    path_option.closest_point = p;
                }
            }
        }
        return;
    }

    Vector2f c = Vector2f(0, 1 / curvature);
    float r_inner = c.norm() - robot_config.width / 2 - robot_config.safety_margin;
    float r_outer = c.norm() + robot_config.width / 2 + robot_config.safety_margin;
    float r_tl = (Vector2f(0, r_inner) - Vector2f(h + robot_config.safety_margin, 0)).norm();
    float r_tr = (Vector2f(0, r_outer) - Vector2f(h + robot_config.safety_margin, 0)).norm();
    float r_br = (Vector2f(0, r_outer) - Vector2f(robot_config.base_link_offset + robot_config.safety_margin, 0)).norm();
    path_option.free_path_length = std::min(M_PI * c.norm(), 5.0);  // some large number
    // float omega = atan2(h, r_inner);

    float theta_br = asin(robot_config.base_link_offset + robot_config.safety_margin / r_br); // angle where back right would hit
    float phi = 0;
//	cout << "curvature " << curvature << endl;
//	bool front_side = false, outer_side = false, inner_side = false;
    for (unsigned int i = 0; i < point_cloud.size(); i++) {
        Vector2f p = point_cloud[i];
        float r_p = (c-p).norm();
        float theta = curvature < 0 ? atan2(p[0], p[1]- c[1]) : atan2(p[0], c[1] - p[1]); // angle between p and c
        float length = 5.0;
        // cout << "curvature " << curvature << endl;
        if (r_inner <= r_p && r_p <= r_tl) {    // inner side hit
                phi = acos(r_inner / r_p);
                length = (theta - phi) * c.norm();
            // inner_side = true;
                // cout << "inner side hit" << endl;
        }
        if ((r_inner <= r_p && r_p <= r_br) && (-theta_br <= theta && theta <= theta_br)) {    // outer side hit
            phi = acos(r_p / (c.norm() + robot_config.width / 2));
            length = (theta - phi) * c.norm();
	    // outer_side = true;
	    // cout << "outer side hit" << endl;
        }

        if (r_tl <= r_p && r_p <= r_tr) {    // front side hit
            phi = asin(h / r_p);
            length = (theta - phi) * c.norm();
	    // front_side = true;
	    // cout << "front side hit" << endl;
        }
        if (length < path_option.free_path_length && length > 0) {
            path_option.free_path_length = length;
            path_option.obstruction = p;
        }
    }
	// if (inner_side)
	//  	cout << "intersecting particle found with inner side" << endl;
	// if (outer_side)
	//	cout << "intersecting particle found with outer side" << endl;
	//if (front_side)
	//	cout << "intersecting particle found with front side" << endl;

    // float theta = M_PI / 2;
    // if (path_option.obstruction != Eigen::Vector2f::Zero()) {
    //     theta = curvature < 0 ? atan2(path_option.obstruction[0], path_option.obstruction[1]- c[1]) :
    //         atan2(path_option.obstruction[0], c[1] - path_option.obstruction[1]);
    // }
    // clearance
    // path_option.clearance = 100; // some large number
    for (auto p: point_cloud) {
        float theta_p =  curvature < 0 ? atan2(p[0], p[1]- c[1]) :
            atan2(p[0], c[1] - p[1]);
        float path_len_p = theta_p * (p-c).norm();
        if (path_len_p >=0 and path_len_p < path_option.free_path_length) {  // if p is within the fp length
            float inner = abs((c - p).norm() - r_inner);
            float outer = abs((c - p).norm() - r_tr);
            float clearance_p = std::min(inner, outer);
            if (clearance_p < path_option.clearance) {
                path_option.clearance = clearance_p;
                path_option.closest_point = p;
            }
        }
    }
}


// sample path options
// given point cloud (robot frame), num options, max curvature
// out const vector path options

vector<navigation::PathOption> samplePathOptions(int num_options,
                                                    const vector<Eigen::Vector2f>& point_cloud,
                                                    const navigation::NavigationParams& robot_config,
                                                    Eigen::Vector2f goal_loc) {
    static vector<navigation::PathOption> path_options;
    path_options.clear();
    float max_curvature = robot_config.max_curvature;

    // loop through curvature from max to -max
    for (int i = 0; i < num_options; i++) { 
        float curvature = max_curvature * pow(2*i/float(num_options-1) - 1, 2);
        if (i < num_options / 2) {
            curvature = -curvature;
        }
        
        navigation::PathOption path_option;
        setPathOption(path_option, curvature, point_cloud, robot_config);

        float turning_radius = 1/curvature;
        float fpl_lim_curve = fabs(turning_radius)*atan2(goal_loc.norm(), fabs(turning_radius));
        float kEpsilon = 1e-5;
        if(curvature < kEpsilon && curvature > -kEpsilon && path_option.free_path_length > goal_loc.norm()){
            // Straight path
            path_option.free_path_length = goal_loc.norm();
        } else if (path_option.free_path_length > fpl_lim_curve) {
            // Using the optimization mentioned in class where we take the free path 
            // only till the tangent.
            path_option.free_path_length = fpl_lim_curve;
        }

        path_options.push_back(path_option);
    }
    // exit(0);
    return path_options;
}


float score(float free_path_length, float curvature, float clearance, Eigen::Vector2f goal_loc) {
    const float w1 = 1;
    const float w2 = 0;
    const float w3 = 0.1;
    const float w4 = -1.0;
    // compute endpoint of arc
    if (fabs(curvature) < 0.05) {
        return w1 * free_path_length + w2 * abs(1/curvature) + w3 * clearance + w4 * (goal_loc - Eigen::Vector2f(free_path_length, 0)).norm();
    }
    float r = 1.0 / curvature;
    float theta = free_path_length / r;
    Eigen::Vector2f endPoint = Eigen::Vector2f(r * sin(theta), r - r * cos(theta));
    return w1 * free_path_length + w2 * abs(1/curvature) + w3 * clearance + w4 * (goal_loc - endPoint).norm();
}

// returns the index of the selected path
// for now, just return the index of the path with the longest free path length
// if there are multiple paths with the same free path length, return the one with the smallest curvature
int selectPath(const vector<navigation::PathOption>& path_options, Eigen::Vector2f goal_loc, float robot_angle, Eigen::Vector2f robot_loc_) {
    int selected_path = 0;
    float best_score = 0;

    // Eigen::Vector2f goal_loc_in_robot_frame;
    // float x = goal_loc.x();
    // float y = goal_loc.y();
    // // convert from global frame to robot frame
    // float translationX = robot_loc_.x();
    // float translationY = robot_loc_.y();
    // float newX = cos(robot_angle) * (x - translationX) + sin(robot_angle) * (y - translationY);
    // float newY = -sin(robot_angle) * (x - translationX) + cos(robot_angle) * (y - translationY);
    // goal_loc_in_robot_frame = Eigen::Vector2f(newX, newY);

    for (unsigned int i = 0; i < path_options.size(); i++) {
        float s = score(path_options[i].free_path_length, path_options[i].curvature, path_options[i].clearance, goal_loc);
        if (s > best_score) {
            best_score = s;
            selected_path = i;
        }
    }
    return selected_path;
}

