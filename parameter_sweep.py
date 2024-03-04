#!/usr/bin/env python3

import subprocess
import re
import random
import os
    
def extract_scores(text):
    # Separate regex patterns to extract both float and integer values for loc_score and angle_score
    pattern_loc = re.compile(r"loc score: (\d+(?:\.\d+)?)")
    pattern_angle = re.compile(r"angle score: (\d+(?:\.\d+)?)")
    
    # Find all matches in the text for loc_score and angle_score separately
    matches_loc = pattern_loc.findall(text)
    matches_angle = pattern_angle.findall(text)
    
    # If matches found, return the last ones as a dictionary with values converted appropriately
    if matches_loc and matches_angle:
        last_loc = matches_loc[-1]
        last_angle = matches_angle[-1]
        return float(last_loc), float(last_angle)
    else:
        return {}
    
def get_random_hyperparam_sample():
    
    # original
    # stddev_bounds = [0.01, 0.3]
    # gamma_bounds = [0.02, 1.0]
    # # d_long_bounds = [0.01, 1]
    # d_long_bounds = [1.0, 1.5]
    # # d_short_bounds = [0.01, 0.5]
    # d_short_bounds = [0.2, 0.5]
    # k1_bounds = [0.01, 0.5]
    # k2_bounds = [0.01, 0.5]
    # k3_bounds = [0.01, 0.5]
    # k4_bounds = [0.01, 0.5]
    
    stddev_bounds = [0.05, 0.3]
    gamma_bounds = [0.1, 0.5]
    d_long_bounds = [1.2, 1.4]
    d_short_bounds = [0.35, 0.5]
    k1_bounds = [0.1, 0.5]
    k2_bounds = [0.1, 0.4]
    k3_bounds = [0, 0.5]
    k4_bounds = [0.1, 0.5]
    
    return {
        'stddev': random.uniform(stddev_bounds[0], stddev_bounds[1]),
        'gamma': random.uniform(gamma_bounds[0], gamma_bounds[1]),
        'd_long': random.uniform(d_long_bounds[0], d_long_bounds[1]),
        'd_short': random.uniform(d_short_bounds[0], d_short_bounds[1]),
        'k1': random.uniform(k1_bounds[0], k1_bounds[1]),
        'k2': random.uniform(k2_bounds[0], k2_bounds[1]),
        'k3': random.uniform(k3_bounds[0], k3_bounds[1]),
        'k4': random.uniform(k4_bounds[0], k4_bounds[1])
    }

def main():
    # Define parameter space
    
    best_loc_score = float('inf')
    best_angle_score = float('inf')
    best_params = None
    best_run = None

    i = 1
    
    rosbag_files_folder = '/home/dev/cs393r_starter/src/particle_filter/bag_data'
    # find all rosbag files in all subdirs using os.walk
    rosbag_files = []
    for root, dirs, files in os.walk(rosbag_files_folder):
        for file in files:
            if file.endswith('.bag'):
                rosbag_files.append(os.path.join(root, file))
    n_episodes = len(rosbag_files)
                
    # at each iteration, append output to a file
    output_data = "tuned_parameters.txt"
    
    if not os.path.exists(output_data):
        # create the file
        with open(output_data, 'w') as file:
            file.write("Run, Loc Score, Angle Score, Parameters, Best Loc Score, Best Angle Score, Best Parameters\n")
    
    while True:
        parameters = get_random_hyperparam_sample()
        print('Running with parameters:', parameters)
        
        # Construct command to call binary script with current parameter combination
        particle_filter_command = ['./bin/particle_filter']
        for param in parameters:
            particle_filter_command.append(f' -{param} ')
            particle_filter_command.append(str(parameters[param]))

        # Call binary script
        particle_filter_process = subprocess.Popen(particle_filter_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        loc_score_cum_sum = 0.0
        angle_score_cum_sum = 0.0
        
        for file_num, rosbag_file in enumerate(rosbag_files):
            print('Running file number:', file_num + 1, 'of', len(rosbag_files))
            # Construct command to run ROS bag file
            rosbag_command = ['rosbag', 'play', rosbag_file]
        
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
            loc_score, angle_score = extract_scores(output_str[-50:])
            loc_score_cum_sum += loc_score
            angle_score_cum_sum += angle_score
        
        total_loc_score = loc_score_cum_sum / n_episodes
        total_angle_score = angle_score_cum_sum / n_episodes
        
        # Update best parameters if current scores are lower than the previous best
        if total_loc_score < best_loc_score and total_angle_score < best_angle_score:
            best_loc_score = total_loc_score
            best_angle_score = total_angle_score
            best_params = parameters
            best_run = i

        print(f"Run ({i})")
        print("Parameter set:")
        print(parameters)
        print(f"Loc Score: {total_loc_score}")
        print(f"Angle Score: {total_angle_score}\n")
    
        # Print the best parameter set
        print(f"Best parameters: {best_params}")
        print(f"Best loc score: {best_loc_score}")
        print(f"Best angle score: {best_angle_score}")
        print(f"Best run = {best_run}\n")
        print("\n")
        
        # write stuff to file
        with open(output_data, 'a') as file:
            file.write(f"{i}, {total_loc_score}, {total_angle_score}, {parameters}, {best_loc_score}, {best_angle_score}, {best_params}\n")

        i += 1




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
