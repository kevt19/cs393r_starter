#!/usr/bin/env python3

import subprocess


def main():
    # Define parameter space
    parameters = {
        'stddev': [0.05, 0.1, 0.3, 0.5, 0.8, 1.0],
        'gamma': [0.00092, 0.001, 0.01, 0.1, 0.5, 0.85, 1.0]
    }
    
    best_score = float('-inf')
    best_params = None

    # Grid search
    for stddev in parameters['stddev']:
        for gamma in parameters['gamma']:
            # Construct command to call binary script with current parameter combination
            particle_filter_command = ['./bin/particle_filter', ' -stddev ', str(stddev), ' -gamma ', str(gamma)]

            # Call binary script
            particle_filter_process = subprocess.Popen(particle_filter_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

            # Construct command to run ROS bag file
            rosbag_command = ['rosbag', 'play', '/home/kevt19/catkin_ws/src/cs393r_starter/src/particle_filter/bag_data/Spring2020/2020-04-01-17-20-04.bag']
            
            # Start ROS bag file
            rosbag_process = subprocess.Popen(rosbag_command)

            # Wait for ROS bag file to finish
            rosbag_process.wait()
        
            # Terminate binary script once ROS bag file is finished
            particle_filter_process.terminate()
            
            # Read the output of the binary process
            stdout, stderr = particle_filter_process.communicate()

            # Decode the output bytes into a string
            output_str = stdout.decode('utf-8')
        
            # Process output to extract score
            score = output_str

            # Convert score to float (assuming it's a numerical value)
            score = float(score)
            
            # Check if current parameter combination yields the best score
            if score > best_score:
                best_score = score
                best_params = {'stddev': stddev, 'gamma': gamma}

    print("Best parameters:", best_params)
    print("Best score:", best_score)



if __name__ == "__main__":
    main()
