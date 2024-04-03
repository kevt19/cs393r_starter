#include <queue>
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "ros/ros.h"
#include "eigen3/Eigen/Dense"

#ifndef LATENCY_COMPENSATION_H
#define LATENCY_COMPENSATION_H

struct Control {
    float x_dot;
    float y_dot;
    float theta_dot;
    double time;
};

struct Observation {
    float x;
    float y;
    float theta;
    float vx;
    float vy;
    float omega;
    double time;
};

class LatencyCompensation {
public:
    LatencyCompensation(float actuation_latency, float observation_latency, double dt) :
        actuation_latency_(actuation_latency),
        observation_latency_(observation_latency),
        dt_(dt) {};
    
    // Mostly for debugging
    float getActuationDelay() {
        return actuation_latency_;
    }

    void setActuationDelay(float actuation_latency) {
        actuation_latency_ = actuation_latency;
    }

    float getObservationDelay() {
        return observation_latency_;
    }

    void setObservationDelay(float observation_latency) {
        observation_latency_ = observation_latency;
    }

    std::queue<Control> getControlQueue() {
        return control_queue_;
    }

    void recordControl(const Control& control) {
        control_queue_.push(control);
    }

    // Store last observed state
    void recordObservation(float x, float y, float theta, double time) {
        last_x_ = x;
        last_y_ = y;
        last_theta_ = theta;
        last_observation_time_ = time - observation_latency_;
    };

    Observation getPredictedState() {
        Observation predictedState = {last_x_, last_y_, last_theta_, 0.0, 0.0, 0.0, last_observation_time_};
	// cout << "last observed state: " << last_observation_.x << " " << last_observation_.y << endl;
        double control_cutoff_time_ = last_observation_time_ - actuation_latency_;
        double current_time = ros::Time::now().toSec() - actuation_latency_;

        while (control_queue_.size() > 0 && control_queue_.front().time < control_cutoff_time_) 
            control_queue_.pop();

        bool current_found = false;
        while (control_queue_.size() > 0) {
	    Control control = control_queue_.front();
            if (!current_found && control.time >= current_time) {
                predictedState.vx = control.x_dot;
                predictedState.vy = control.y_dot;
                predictedState.omega = control.theta_dot;
                current_found = true;
            }
            predictedState.x += control.x_dot * dt_;
            predictedState.y += control.y_dot * dt_;
            predictedState.theta += control.theta_dot * dt_;
            control_queue_.pop();
        }

	// cout << "predicted state: " << predictedState.x << " " << predictedState.y << endl;
        return predictedState;
    }

    std::vector<Eigen::Vector2f> forward_predict_point_cloud(const std::vector<Eigen::Vector2f>& point_cloud, float predicted_x, float predicted_y, float predicted_theta) {
        float x_shift = predicted_x - last_x_;
        float y_shift = predicted_y - last_y_;
        float theta_shift = predicted_theta - last_theta_;
        std::vector<Eigen::Vector2f> predicted_point_cloud;
        for (auto p: point_cloud) {
            float x = p[0] * cos(theta_shift) + p[1] * sin(theta_shift) - x_shift;
            float y = -1 * p[0] * sin(theta_shift) + p[1] * cos(theta_shift) - y_shift;
            predicted_point_cloud.push_back(Eigen::Vector2f(x, y));
        }
        return predicted_point_cloud;
    }

private:
    float actuation_latency_;
    float observation_latency_;
    double dt_;
    std::queue<Control> control_queue_;
    
    float last_x_;
    float last_y_;
    float last_theta_;
    double last_observation_time_;
};

#endif  // LATENCY_COMPENSATION_H
