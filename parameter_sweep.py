#!/usr/bin/env python3

import subprocess
import re

def extract_scores(output_str):
    loc_score_match = re.search(r'loc score: (\d+\.\d+)', output_str)
    angle_score_match = re.search(r'angle score: (\d+\.\d+)', output_str)
    
    if loc_score_match and angle_score_match:
        loc_score = float(loc_score_match.group(1))
        angle_score = float(angle_score_match.group(1))
        return loc_score, angle_score
    else:
        return None, None

def main():
    # # Define parameter space
    # parameters = {
    #     'stddev': [0.05, 0.5, 0.8, 1.0],
    #     'gamma': [0.02, 0.1, 0.5, 1.0],
    #     'd_long': [1, 1.5, 2, 2.5],
    #     'd_short': [0.2, 0.3, 0.4, 0.5]
    # }
    # # Calculate the number of parameter combinations
    # num_combinations = 1
    # for param_values in parameters.values():
    #     num_combinations *= len(param_values)
    
    # best_loc_score = float('inf')
    # best_angle_score = float('inf')
    # best_params = None
    # best_run = None

    # i = 1
    # # Grid search
    # for stddev in parameters['stddev']:
    #     for gamma in parameters['gamma']:
    #         for d_long in parameters['d_long']:
    #             for d_short in parameters['d_short']:
                    
    #                 # Construct command to call binary script with current parameter combination
    #                 particle_filter_command = ['./bin/particle_filter', ' -stddev ', str(stddev), ' -gamma ', str(gamma), ' -d_long ', str(d_long), ' -d_short ', str(d_short)]

    #                 # Call binary script
    #                 particle_filter_process = subprocess.Popen(particle_filter_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    #                 # Construct command to run ROS bag file
    #                 rosbag_command = ['rosbag', 'play', '/home/kevt19/catkin_ws/src/cs393r_starter/src/particle_filter/bag_data/Spring2020/2020-04-01-17-20-04.bag']
                    
    #                 # Start ROS bag file
    #                 rosbag_process = subprocess.Popen(rosbag_command)

    #                 # Wait for ROS bag file to finish
    #                 rosbag_process.wait()
                
    #                 # Terminate binary script once ROS bag file is finished
    #                 particle_filter_process.terminate()
                    
    #                 # Read the output of the binary process
    #                 stdout, stderr = particle_filter_process.communicate()

    #                 # Decode the output bytes into a string
    #                 output_str = stdout.decode('utf-8')

    #                 # print(output_str)
                
    #                 # Process output to extract score
    #                 loc_score, angle_score = extract_scores(output_str[-50:])
                    
    #                 print("\n")
    #                 print(f"Run ({i}/{num_combinations})")
    #                 print(f"Time elapsed = {i*29.6516} Seconds / {num_combinations*29.6516} Seconds\n")

    #                 print("Parameter set:")
    #                 print(f"stddev = {stddev}")
    #                 print(f"gamma = {gamma}\n")
    #                 print(f"d_long = {d_long}")
    #                 print(f"d_short = {d_short}\n")

    #                 print(f"Loc Score: {loc_score}")
    #                 print(f"Angle Score: {angle_score}\n")
                    
    #                 # Update best parameters if current scores are lower than the previous best
    #                 if loc_score < best_loc_score and angle_score < best_angle_score:
    #                     best_loc_score = loc_score
    #                     best_angle_score = angle_score
    #                     best_params = {'stddev': stddev, 'gamma': gamma, 'd_long': d_long, 'd_short': d_short}
    #                     best_run = i

    #                 # Print the best parameter set
    #                 print(f"Best parameters: {best_params}")
    #                 print(f"Best loc score: {best_loc_score}")
    #                 print(f"Best angle score: {best_angle_score}")
    #                 print(f"Best run = {best_run}\n")
    #                 print("\n")


    #                 i += 1




    # Define parameter space
    parameters = {
    'k1': [0.1, 0.2, 0.3, 0.4],
    'k2': [0.1, 0.2, 0.3, 0.4],
    'k3': [0.15, 0.3, 0.45, 0.60],
    'k4': [0.15, 0.3, 0.45, 0.60]
    }
    # Calculate the number of parameter combinations
    num_combinations = 1
    for param_values in parameters.values():
        num_combinations *= len(param_values)
    
    best_loc_score = float('inf')
    best_angle_score = float('inf')
    best_params = None
    best_run = None

    i = 1
    # Grid search
    for k1 in parameters['k1']:
        for k2 in parameters['k2']:
            for k3 in parameters['k3']:
                for k4 in parameters['k4']:
                    
                    # Construct command to call binary script with current parameter combination
                    particle_filter_command = ['./bin/particle_filter', ' -k1 ', str(k1), ' -k2 ', str(k2), ' -k3 ', str(k3), ' -k4 ', str(k4)]

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

                    # print(output_str)
                
                    # Process output to extract score
                    loc_score, angle_score = extract_scores(output_str[-50:])
                    
                    print("\n")
                    print(f"Run ({i}/{num_combinations})")
                    print(f"Time elapsed = {i*29.6516} Seconds / {num_combinations*29.6516} Seconds\n")

                    print("Parameter set:")
                    print(f"k1 = {k1}")
                    print(f"k2 = {k2}\n")
                    print(f"k3 = {k3}")
                    print(f"k4 = {k4}\n")

                    print(f"Loc Score: {loc_score}")
                    print(f"Angle Score: {angle_score}\n")
                    
                    # Update best parameters if current scores are lower than the previous best
                    if loc_score < best_loc_score and angle_score < best_angle_score:
                        best_loc_score = loc_score
                        best_angle_score = angle_score
                        best_params = {'k1': k1, 'k2': k2, 'k3': k3, 'k4': k4}
                        best_run = i

                    # Print the best parameter set
                    print(f"Best parameters: {best_params}")
                    print(f"Best loc score: {best_loc_score}")
                    print(f"Best angle score: {best_angle_score}")
                    print(f"Best run = {best_run}\n")
                    print("\n")


                    i += 1


if __name__ == "__main__":
    main()
