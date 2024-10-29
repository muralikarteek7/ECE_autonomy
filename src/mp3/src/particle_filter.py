import numpy as np
from maze import Maze, Particle, Robot
import bisect
import rospy
from gazebo_msgs.msg import  ModelState
from gazebo_msgs.srv import GetModelState
import shutil
from std_msgs.msg import Float32MultiArray
from scipy.integrate import ode

import random

def vehicle_dynamics(t, vars, vr, delta):
    curr_x = vars[0]
    curr_y = vars[1] 
    curr_theta = vars[2]
    
    dx = vr * np.cos(curr_theta)
    dy = vr * np.sin(curr_theta)
    dtheta = delta
    return [dx,dy,dtheta]

class particleFilter:
    def __init__(self, bob, world, num_particles, sensor_limit, x_start, y_start):
        self.num_particles = num_particles  # The number of particles for the particle filter
        self.sensor_limit = sensor_limit    # The sensor limit of the sensor
        particles = list()

        ##### TODO:  #####
        # Modify the initial particle distribution to be within the top-right quadrant of the world, and compare the performance with the whole map distribution.
        for i in range(num_particles):

            # (Default) The whole map
            x = np.random.uniform(0, world.width)
            y = np.random.uniform(0, world.height)


            ## first quadrant
            # x = np.random.uniform(85, 85+120)
            # y = np.random.uniform(45-75, 45)
            x = np.random.uniform(world.width/2, world.width)
            y = np.random.uniform(world.height/2, world.width)

            particles.append(Particle(x = x, y = y, maze = world, sensor_limit = sensor_limit))

        ###############

        self.particles = particles          # Randomly assign particles at the begining
        self.bob = bob                      # The estimated robot state
        self.world = world                  # The map of the maze
        self.x_start = x_start              # The starting position of the map in the gazebo simulator
        self.y_start = y_start              # The starting position of the map in the gazebo simulator
        self.modelStatePub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.controlSub = rospy.Subscriber("/gem/control", Float32MultiArray, self.__controlHandler, queue_size = 1)
        self.control = []                   # A list of control signal from the vehicle
        return

    def __controlHandler(self,data):
        """
        Description:
            Subscriber callback for /gem/control. Store control input from gem controller to be used in particleMotionModel.
        """
        tmp = list(data.data)
        self.control.append(tmp)

    def getModelState(self):
        """
        Description:
            Requests the current state of the polaris model when called
        Returns:
            modelState: contains the current model state of the polaris vehicle in gazebo
        """

        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            modelState = serviceResponse(model_name='polaris')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
        return modelState

    def weight_gaussian_kernel(self,x1, x2, std = 5000):
        if x1 is None: # If the robot recieved no sensor measurement, the weights are in uniform distribution.
            print("robot no sensor")
            return 1./len(self.particles)
        else:
            tmp1 = np.array(x1)
            tmp2 = np.array(x2)
            return np.sum(np.exp(-((tmp2-tmp1) ** 2) / (2 * std)))


    def updateWeight(self, readings_robot):
        """
        Description:
            Update the weight of each particles according to the sensor reading from the robot 
        Input:
            readings_robot: List, contains the distance between robot and wall in [front, right, rear, left] direction.
        """

        ## TODO #####
        weight_array = []
        for particle in self.particles:
            particle_reading = particle.read_sensor()
            weight = self.weight_gaussian_kernel(readings_robot, particle_reading)   #compute weights
            weight_array.append(weight)                                         #add weight to array

        weight_array = np.array(weight_array)
        weight_sum = np.sum(weight_array)
        normalized = np.divide(weight_array, weight_sum)
        for i in range(len(self.particles)):
            self.particles[i].weight = normalized[i]    # set each particle weight to corresponding normalized weight
        # print(normalized)
        ###############
        # pass

    def resampleParticle(self):
        """
        Description:
            Perform resample to get a new list of particles 
        """
        particles_new = list()
        ## TODO #####

        # calculate cumulative sums
        particle_weights =[]
        for i in range(len(self.particles)):
            if(i == 0):
                particle_weights.append(self.particles[i].weight)
                continue
            #add particle weight to previously calculated weights to get cumulative sum
            particle_weights.append(particle_weights[i-1]+self.particles[i].weight) 
        # generate random nums and particle index 
        for i in range(len(self.particles)):      
            rand_num = random.uniform(0,1)

            #index of where num could be inserted to maintain order
            ind = np.searchsorted(particle_weights, rand_num, side = 'left')
            particles_new.append(self.particles[ind])
        ###############

        self.particles = particles_new

    def particleMotionModel(self):
        """
        Description:
            Estimate the next state for each particle according to the control input from actual robot 
            You can either use ode function or vehicle_dynamics function provided above
        """
        ## TODO #####
        particles = self.particles
        control_signal = self.control
        dt = 0.01
        t = 0
        # for control_input in control_signal:

        # currState = getModelState()
        # pos_x = currState.pose.position.x
        # pos_y = currState.pose.position.y
        # vel = currentPose.twist.linear
        # def vehicle_dynamics(t, vars, vr, delta):

        for particle in particles:
            vehicle_dynam = ode(vehicle_dynamics).set_integrator('vode')
            state = particle.state #in maze.py Particle class

            #ode method
            # vehicle_dynam.set_initial_value([state[0],state[1],state[2]], t)
            # for v, delta in control_signal:
            #     vehicle_dynam.set_initial_value([state[0],state[1],state[2]], t)
            #     vehicle_dynam.set_f_params(v,delta)
            #     vehicle_dynam.integrate(vehicle_dynam.t + dt)
            #     state = vehicle_dynam.y         #returns updated state after integrating over time setep
            #     t += dt
            # particle.x = state[0]
            # particle.y = state[1]
            # particle.heading = state[2]
            
            # vehicle dynamic method
            for v, delta in control_signal:
                dx,dy,dtheta = vehicle_dynamics(t, [state[0],state[1],state[2]], v, delta)
                particle.try_move([dx, dy, dtheta], self.world)
                # particle.x += dx
                # particle.y += dy
                # particle.heading += dtheta
                t += dt


        
        ###############
        # pass
        return


    def runFilter(self):
        """
        Description:
            Run PF localization
        """
        count = 0 
        while True:
            ## TODO: (i) Implement Section 3.2.2. (ii) Display robot and particles on map. (iii) Compute and save position/heading error to plot. #####

            self.particleMotionModel()
            reading = self.bob.read_sensor()
            self.updateWeight(reading)
            self.resampleParticle()

            self.world.show_particles(self.particles)
            self.world.show_robot(self.bob)
            self.world.show_estimated_location(self.particles)
            
            rate = rospy.Rate(100)  # 100 Hz
            rate.sleep()
            ###############
            # return
