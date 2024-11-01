import numpy as np
from maze import Maze, Particle, Robot
import bisect
import rospy
from gazebo_msgs.msg import  ModelState
from gazebo_msgs.srv import GetModelState
import shutil
from std_msgs.msg import Float32MultiArray
from scipy.integrate import ode
import time
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
            # x = np.random.uniform(world.width/2, world.width)
            # y = np.random.uniform(world.height/2, world.height)
            # x = 85+15
            # y = 45


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
        # particles_readings = []
        count = len(self.particles)
        for particle in self.particles:
            particle_reading = particle.read_sensor()
            # print(particle_reading)
            # particles_readings.append(particle_reading)
            if (0 in particle_reading):
                # np.append(weight_array,0)
                weight_array.append(0)
                continue
            weight = self.weight_gaussian_kernel(readings_robot, particle_reading, count*10)   #compute weights
            # np.append(weight_array, weight)                                     #add weight to array
            weight_array.append(weight)
        # print("Robot: ",readings_robot)
        weight_array = np.array(weight_array)
        weight_sum = np.sum(weight_array)
        # print(weight_sum)
        
        normalized = np.divide(weight_array, weight_sum)
        # print(normalized)

        # for i in range(len(self.particles)):
        #     self.particles[i].weight = normalized[i]    # set each particle weight to corresponding normalized weight
        return normalized
        # print(normalized)
        
        # ind_max = np.argmax(normalized)
        # print("=================================")        
        # # print("robot: ", readings_robot)
        # print("particles reading: ",particles_readings[ind_max])
        # if(not np.array(particles_readings[ind_max]).any()):
        #     print("zero")
        # print("weight: ", normalized[ind_max])

        ###############
        # pass

    def resampleParticle(self, weight):
        """
        Description:
            Perform resample to get a new list of particles 
        """
        particles_new = list()
        ## TODO #####

        # calculate cumulative sums
        # particle_weights =[]
        # for i in range(len(self.particles)):
        #     if(i == 0):
        #         particle_weights.append(self.particles[i].weight)
        #         continue
        #     #add particle weight to previously calculated weights to get cumulative sum
        #     particle_weights.append(particle_weights[i-1]+self.particles[i].weight) 
        #     # particle_weights.append(self.particles[i].weight)
        # # generate random nums and particle index 
        # indicies = []
        # particle_weights = np.array(particle_weights)
        # # rand_num_lst = np.random.uniform(0,1,len(self.particles))
        # for i in range(len(self.particles)):      
        #     rand_num = random.uniform(0,1)
        #     rand_num = (i+rand_num) / len(self.particles)  
        #     # rand_num = rand_num_lst[i]
        #     #index of where num could be inserted to maintain order
        #     ind = np.searchsorted(particle_weights, rand_num, side = 'left') 
        #     indicies.append(ind)
        #     # print(ind)

        #     particles_new.append(Particle(x = self.particles[ind].x, y = self.particles[ind].y, heading = self.particles[ind].heading, maze = self.particles[ind].maze, weight = self.particles[ind].weight, sensor_limit = self.particles[ind].sensor_limit, noisy = True))
        #     # new_part = Particle(x = self.particles[ind].x, y = self.particles[ind].y, maze = self.particles[ind].maze, weight = self.particles[ind].weight, sensor_limit = self.particles[ind].sensor_limit)

        #     # particles_new.append(new_part)
        
        # weight = np.array(weight)
        new_sample_part = np.random.choice(a=self.particles, size=len(self.particles), replace=True, p=weight)
        self.particles = []
        for ind in range(len(new_sample_part)):      
            # new_part = Particle(x = new_sample_part[ind].x, y = new_sample_part[ind].y, heading = new_sample_part[ind].heading, maze = new_sample_part[ind].maze, sensor_limit = new_sample_part[ind].sensor_limit)

            new_part = Particle(x = new_sample_part[ind].x, y = new_sample_part[ind].y, heading = new_sample_part[ind].heading, maze = new_sample_part[ind].maze, sensor_limit = new_sample_part[ind].sensor_limit,  noisy = True)

            # particles_new.append(new_part)
            self.particles.append(new_part)
        ###############
        
        # self.particles = particles_new

    def particleMotionModel(self):
        """
        Description:
            Estimate the next state for each particle according to the control input from actual robot 
            You can either use ode function or vehicle_dynamics function provided above
        """
        ## TODO #####
        # print("update particle motion")
        particles = self.particles
        control_signal = self.control
        dt = 0.015 #1500
        dt = 0.0105    #1000
        # dt = 0.01
        t = 0
        # vehicle_dynam = ode(vehicle_dynamics).set_integrator('dopri5')

        for particle in particles:
        # for v, delta in self.control:
            # t = 0
            # # state = particle.state #in maze.py Particle class
            # # print("========================================")
            # # ode method
            # # vehicle_dynam.set_initial_value([state[0],state[1],state[2]], t)
            # vehicle_dynam.set_initial_value([particle.x, particle.y , particle.heading], t)
            # # vehicle_dynam.set_f_params(v,delta)
            # for v, delta in control_signal:
            # # for particle in self.particles:
            #     # state = particle.state 
            #     # vehicle_dynam.set_initial_value([particle.x, particle.y , particle.heading], t)
            #     # vehicle_dynam.set_initial_value([state[0],state[1],state[2]], t)
            #     vehicle_dynam.set_f_params(v,delta)
            #     vehicle_dynam.integrate(vehicle_dynam.t + dt)
            #     # state = vehicle_dynam.y         #returns updated state after integrating over time setep
            #     particle.x = vehicle_dynam.y[0]
            #     particle.y = vehicle_dynam.y[1]
            #     particle.heading = vehicle_dynam.y[2]
            #     # print(particle.x)
            #     # print(particle.y)
            #     # print(particle.heading)
            #     t += dt
            # particle.x = state[0]
            # particle.y = state[1]
            # particle.heading = state[2]
            # particle.try_move([particle.x - state[0], particle.y - state[1], particle.heading - state[2]], self.world)
            
            # vehicle dynamic method
            # x = state[0]
            # y = state[1]
            # theta = state[2]
            
            for v, delta in control_signal:
            # for particle in particles:
                # print(particle.state)
                dx,dy,dtheta = vehicle_dynamics(t, [particle.x, particle.y, particle.heading], v, delta)
                particle.try_move([dx*dt, dy*dt, dtheta*dt], self.world)
                # state = particle.state
                # x += dx * dt
                # y += dy * dt
                # theta += dtheta * dt
                # particle.x += dx * dt
                # particle.y += dy * dt
                # particle.heading += (dtheta) * dt
            # t += dt
            # # particle.x = x
            # # particle.y = y
            # # particle.heading = theta
        self.control = []

        
        ###############
        # pass
        # return


    def runFilter(self):
        """
        Description:
            Run PF localization
        """
        count = 0 
        error = []
        orientation_err = []
        self.world.clear_objects()
        self.world.show_particles(self.particles, show_frequency=30)
        self.world.show_robot(self.bob)
        while True:
            ## TODO: (i) Implement Section 3.2.2. (ii) Display robot and particles on map. (iii) Compute and save position/heading error to plot. #####
                        
            # if(count % 2 == 0):
        

            reading = self.bob.read_sensor()

            if(len(self.control) == 0):
                continue
            # # print("updating")

            if(reading is None):
            #     print("reading 0")
                continue

            self.particleMotionModel()
            weight = self.updateWeight(reading)
            self.resampleParticle(weight)
            if(count % 2 == 0):
                self.world.clear_objects()
                self.world.show_particles(self.particles, show_frequency=30)
                self.world.show_robot(self.bob)
                estimated_location = self.world.show_estimated_location(self.particles)
            
            car_state = self.bob.getModelState()


            euclidian_dist = np.sqrt((car_state.pose.position.x - estimated_location[0])**2 + (car_state.pose.position.y - estimated_location[1])**2)
            _,_ , yaw = quaternion_to_euler(x=car_state.pose.orientation.x, y=car_state.pose.orientation.y, z=car_state.pose.orientation.z, w=car_state.pose.orientation.w)
            orientation = np.abs(yaw - estimated_location[2])
            
            #appending errors to lists
            error.append(euclidian_dist)
            orientation_err.append(orientation)


            count += 1
            # if(count == 2):
            #     break
            ###############
            # return
            # 96,47]
            distToTargetX = abs(car_state.pose.position.x - (96 + 15 - 100))
            distToTargetY = abs(car_state.pose.position.y - 47)

            if(distToTargetX < 1 and distToTargetY < 1):
                break
        return error, orientation_err, count
    

def quaternion_to_euler(x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = np.arcsin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)
        return [roll, pitch, yaw]
