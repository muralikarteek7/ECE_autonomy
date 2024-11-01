import numpy as np 
import turtle
import argparse
import time
import pickle
import rospy
from gazebo_msgs.msg import  ModelState
from gazebo_msgs.srv import GetModelState
from maze import Maze, Robot
from particle_filter import particleFilter
import matplotlib.pyplot as plt

def main(window_width, window_height, num_particles, sensor_limit, measurement_noise):
    rospy.init_node("navigator")

    window = turtle.Screen()
    window.setup (width = window_width, height = window_height)

    # Creating the python map for the ECEB environment
    maze = np.zeros((200,200))
    with open('obstacle_list.data', 'rb') as filehandle:
        # read the data as binary data stream
        obstacle = pickle.load(filehandle)

    for (x,y) in obstacle:
        maze[y+100,x+100] = 1

    y_start = 100
    x_start = 15
    width = 120
    height = 75
    maze_ted = np.zeros((height,width),dtype = np.int8)
    for i in range(y_start,y_start+height):
        for j in range(x_start,x_start+width):
            if maze[i,j] == 1:
                    maze_ted[i-y_start,j-x_start] |= 15
            else:
                if(i == 0):
                    maze_ted[i-y_start,j-x_start] |= 1
                elif(i == maze.shape[1]-1):
                    maze_ted[i-y_start,j-x_start] |= 4
                else:
                    if maze[i+1,j] == 1:
                        maze_ted[i-y_start,j-x_start] |= 4
                    if maze[i-1,j] == 1:
                        maze_ted[i-y_start,j-x_start] |= 1
                
                if(j == 0):
                    maze_ted[i-y_start,j-x_start] |= 8
                elif(j == maze.shape[1]-1):
                    maze_ted[i-y_start,j-x_start] |= 2
                else:
                    if maze[i,j+1] == 1:
                        maze_ted[i-y_start,j-x_start] |= 2
                    if maze[i,j-1] == 1:
                        maze_ted[i-y_start,j-x_start] |= 8
    world = Maze(maze = maze_ted, x_start = x_start, y_start = y_start)
    time.sleep(1)
    world.show_maze()

    errors = []
    orientation_errors = []
    errors_avg = []
    orientation_errors_avg = []
    
    for x in range(3):
        print(f'Loop iteration {x}')
        bob = Robot(x = 0, y = 0,heading = 0, maze = world, sensor_limit = sensor_limit, measurement_noise = measurement_noise)
    
        # Run PF localization
        pf = particleFilter(bob = bob, world = world, num_particles = num_particles, sensor_limit = sensor_limit,
                            x_start = x_start, y_start = y_start)
        
        error, orientation_err, count = pf.runFilter()

        
        errors.append(error)
        orientation_errors.append(orientation_err)
        # print(error)
        # print(f'The count is {count}')
        # print(orientation_err)
        # Store errors
        errors_avg.append(np.mean(error))
        orientation_errors_avg.append(np.mean(orientation_err))

    # Calculate average errors
    avg_position_error = np.mean(errors_avg)
    avg_orientation_error = np.mean(orientation_errors_avg)

    # Plotting results with average lines
    plt.figure(figsize=(20, 10))

    # Plot position error with average line
    plt.subplot(2, 2, 1)
    plt.plot(errors_avg, label="Position Error")
    plt.axhline(y=avg_position_error, color='r', linestyle='--', label=f"Avg Position Error = {avg_position_error:.2f}")
    plt.xlabel("Iterations")
    plt.ylabel("Position Error")
    plt.title("Position Error with Average Line")
    plt.legend()

    # Plot orientation error with average line
    plt.subplot(2, 2, 2)
    plt.plot(orientation_errors_avg, label="Orientation Error")
    plt.axhline(y=avg_orientation_error, color='r', linestyle='--', label=f"Avg Orientation Error = {avg_orientation_error:.2f}")
    plt.xlabel("Iterations")
    plt.ylabel("Orientation Error")
    plt.title("Orientation Error with Average Line")
    plt.legend()


    min_length = min(len(run) for run in errors + orientation_errors)
    ee = np.array([np.array(run[:min_length]) for run in errors])
    oo = np.array([np.array(run[:min_length]) for run in orientation_errors])

    e = np.mean(ee, axis=0)
    o = np.mean(oo, axis=0)

    # plot with many line plots of each iteration of errors
    plt.subplot(2, 2, 3)
    for i, error in enumerate(errors):
        plt.plot(error, label=f"Run {i+1}")
    plt.plot(e, label="Average")
    plt.xlabel("Iterations")
    plt.axhline(y=avg_position_error, color='r', linestyle='--', label=f"Avg Position Error = {avg_position_error:.2f}")
    plt.ylabel("Position Error")
    plt.title("Position Error for Each Run")
    plt.legend()


    # estimation position erros with many pline plots of each iteration 
    plt.subplot(2, 2, 4)
    for i, orientation_err in enumerate(orientation_errors):
        plt.plot(orientation_err, label=f"Run {i+1}")
    plt.plot(o, label="Average")
    plt.xlabel("Iterations")
    plt.ylabel("Orientation Error")
    plt.axhline(y=avg_orientation_error, color='r', linestyle='--', label=f"Avg Orientation Error = {avg_orientation_error:.2f}")
    plt.title("Orientation Error for Each Run")
    plt.legend()

    plt.tight_layout()
    plt.savefig("position_orientation_error.png")
    plt.show()



    # errorlist = []
    # orientation_errorlist = []
    # arraylist = []
    
    # for i in range(1):
    #     error = []
    #     bob = Robot(x = 0, y = 0,heading = 0, maze = world, sensor_limit = sensor_limit, measurement_noise = measurement_noise)

    #     # Run PF localization
    #     pf = particleFilter(bob = bob, world = world, num_particles = num_particles, sensor_limit = sensor_limit,
    #                         x_start = x_start, y_start = y_start)
        
    #     error, orientation_err, count = pf.runFilter()
    #     print(count)
       
    #     errorlist.append(error)
    #     orientation_errorlist.append(orientation_err)
    #     array = [x for x in range(0, count)]
    #     arraylist.append(array)

    # # Plotting average position and orientation error over iterations
    # avg_position_error = np.mean(errorlist, axis=0)
    # avg_orientation_error = np.mean(orientation_errorlist, axis=0)
    # iterations = arraylist[0]  

    # plt.figure()
    # plt.plot(iterations, avg_position_error, label="Position Error")
    # plt.plot(iterations, error, label="Position Error")
    # plt.plot(iterations, avg_orientation_error, label="Orientation Error")
    # plt.xlabel("Iterations")
    # plt.ylabel("Error")
    # plt.title("Error in Position and Orientation Estimation")
    # plt.legend()
    # plt.savefig("position_orientation_error.png")
    # plt.show()
    # np.save("position_error.npy", avg_position_error)
    # np.save("orientation_error.npy", avg_orientation_error)


    """

    plt.plot(array, error)

    plt.savefig("error.png")
    np.save("error.npy", error)
    np.save("array.npy", array)
    # print(error)
    """

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description = 'Particle filter in maze.')

    # The size of the python map
    window_width_default = 1200
    window_height_default = 750

    # Default values for the parameters for particle filter
    num_particles_default = 1000
    sensor_limit_default = 15
    measurement_noise_default = False
    
    
    parser.add_argument('--num_particles', type = int, help = 'Number of particles used in particle filter.', default = num_particles_default)
    parser.add_argument('--sensor_limit', type = float, help = 'The distance in Gazebo the sensor can sense. ', default = sensor_limit_default)
    parser.add_argument('--measurement_noise', type = bool, help = 'Adding noise to the lidar measurement. ', default = measurement_noise_default)

    
    argv = parser.parse_args()

    window_width = window_width_default
    window_height = window_height_default
    num_particles = argv.num_particles
    sensor_limit = argv.sensor_limit
    measurement_noise = argv.measurement_noise
    
    main(window_width = window_width, window_height = window_height, num_particles = num_particles, sensor_limit = sensor_limit, measurement_noise = measurement_noise)



    # min_length = min(len(errors) for errors in errorlist)
    # truncated_errors = [errors[:min_length] for errors in errorlist]
    # truncated_arrays = [arrays[:min_length] for arrays in arraylist]

    # # Calculate the average error for each iteration
    # average_errors = np.mean(truncated_errors, axis=0)

    # # Prepare x-axis values (assuming all arrays have the same length now)
    # x_values = truncated_arrays[0]

    # # Plotting the results
    # average_errors = np.mean(truncated_errors, axis=0)

    # # Prepare x-axis values (assuming all arrays have the same length now)
    # x_values = truncated_arrays[0]

    # # Create the first plot for average error
    # plt.figure(figsize=(12, 6))

    # plt.subplot(1, 2, 1)  # 1 row, 2 columns, first subplot
    # plt.plot(x_values, average_errors, marker='o')
    # plt.title('Average Error vs Iteration')
    # plt.xlabel('Iteration')
    # plt.ylabel('Average Error')
    # plt.grid()
    


    # # Create the second plot for iteration counts
    # plt.subplot(1, 2, 2)  # 1 row, 2 columns, second subplot
    # plt.plot(x_values, np.arange(len(x_values)), marker='o')
    # plt.title('Iteration Counts')
    # plt.xlabel('Iteration')
    # plt.ylabel('Count')
    # plt.grid()

    # plt.tight_layout()  # Adjust layout to prevent overlap
    # plt.savefig("array.png")
    # plt.show()

