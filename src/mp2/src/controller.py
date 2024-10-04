import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from std_msgs.msg import Float32MultiArray
import math
from util import euler_to_quaternion, quaternion_to_euler
import time
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point



def calculate_curvature(x1, y1, x2, y2,curr_yaw):
        # Calculate the curvature using three consecutive points
        # Points: (x1, y1), (x2, y2), (x3, y3)
        dx1 = x1
        dy1 = y1
        dx2 = x2
        dy2 = y2
        
        angle1 = math.atan2(dy1, dx1)
        angle2 = math.atan2(dy2, dx2)
        
        # The curvature is proportional to the change in angle
        
        return angle1,angle2

class vehicleController():

    def __init__(self):
        # Publisher to publish the control input to the vehicle model
        self.controlPub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size = 1)
        self.futurepoints = rospy.Publisher('future_waypoints', Marker, queue_size=10)
        self.prev_vel = 0
        self.L = 1.75 # Wheelbase, can be get from gem_control.py
        self.log_acceleration = False

        
        self.prev_error = 0.0
        self.integral = 0.0
        self.Kp = 1.0  # Proportional gain
        self.Ki = 0.0  # Integral gain
        self.Kd = 0.1  # Derivative gain
        self.dt = 0.1  # Time step


    def getModelState(self):
        # Get the current state of the vehicle
        # Input: None
        # Output: ModelState, the state of the vehicle, contain the
        #   position, orientation, linear velocity, angular velocity
        #   of the vehicle
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp = serviceResponse(model_name='gem')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
            resp = GetModelStateResponse()
            resp.success = False
        return resp


    # Tasks 1: Read the documentation https://docs.ros.org/en/fuerte/api/gazebo/html/msg/ModelState.html
    #       and extract yaw, velocity, vehicle_position_x, vehicle_position_y
    # Hint: you may use the the helper function(quaternion_to_euler()) we provide to convert from quaternion to euler
    
    
        
    
    def extract_vehicle_info(self, currentPose):

        ####################### TODO: Your TASK 1 code starts Here #######################
        pos_x, pos_y, vel, yaw = 0, 0, 0, 0
        pos_x = currentPose.pose.position.x
        pos_y = currentPose.pose.position.y

        vel_x = currentPose.twist.linear.x
        vel_y = currentPose.twist.linear.y
        vel = math.sqrt(vel_x ** 2 + vel_y ** 2)


        _,_ , yaw = quaternion_to_euler(currentPose.pose.orientation.x,currentPose.pose.orientation.y,currentPose.pose.orientation.z,currentPose.pose.orientation.w)
        
        return pos_x, pos_y, vel, yaw



    def longititudal_controller(self, curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints):

        ####################### TODO: Your TASK 2 code starts Here #######################
        target_velocity = 10


        ####################### TODO: Your TASK 2 code ends Here #######################
        
        high_speed = 20
        mid_speed= 18  # for straight sections
        low_speed = 8.0   # for turns
        #print(curr_vel,len(future_unreached_waypoints))
        # If there are less than 3 waypoints, we can't calculate curvature
        if len(future_unreached_waypoints) < 2:
            return high_speed  # Default to high speed if insufficient waypoints
        
        # Extract the current and future waypoints
        #print(future_unreached_waypoints[0],future_unreached_waypoints[1])
        x1, y1 = future_unreached_waypoints[0]
        x2, y2 = future_unreached_waypoints[1]
        #x3, y3 = future_unreached_waypoints[2]


        y2 =y2 - y1
        x2 =x2 - x1


        print(x1,curr_x, y1, curr_y,math.atan2(y1 - curr_y, x1 - curr_x)*180/3.14 )

        x1 =x1 - curr_x
        y1 =y1 - curr_y


        
        # Calculate the curvature based on the next 3 waypoints
        curvature, curvature1 = calculate_curvature(x1, y1, x2, y2, curr_yaw)
        
        curvature = abs(abs(curvature) - abs(curr_yaw))
        curvature1 = abs((curvature1) - abs(curr_yaw))
        print(curvature*180/3.14,curvature1*180/3.14, curr_yaw*180/3.14)
        print(curvature,curvature1, curr_yaw)
        

        if abs(curvature) <0.1 and abs(curvature1) < 0.1:  # Threshold for curvature, adjust as needed
            target_vel = high_speed  # Straight path
            print("high speed")

        elif abs(curvature) <0.1 and abs(curvature1) > 0.1:
            target_vel = mid_speed
            print("mid speed")
        else:
            target_vel = low_speed   # Shalen(future_unreached_waypoints) > 94rp turn
            print("low speed")
        return target_vel
        #return target_velocity


    # Task 3: Lateral Controller (Pure Pursuit)
    def pure_pursuit_lateral_controller(self, curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints):

        ####################### TODO: Your TASK 3 code starts Here #######################
        # x,y = target_point
        x,y = future_unreached_waypoints[0]
        if len(future_unreached_waypoints) > 1:
            x = (x + future_unreached_waypoints[1][0])/2
            y = (y + future_unreached_waypoints[1][1])/2
        dy = y - curr_y
        dx = x - curr_x
        theta = math.atan2(dy, dx)
        alpha  = theta - curr_yaw
        ld = math.sqrt(dy**2+dx**2)
        target_steering = math.atan2(2*self.L*math.sin(alpha), ld)

        ####################### TODO: Your TASK 3 code starts Here #######################
        return target_steering
        
        
        #return target_steering


    def execute(self, currentPose, target_point, future_unreached_waypoints):
        # Compute the control input to the vehicle according to the
        # current and reference pose of the vehicle
        # Input:
        #   currentPose: ModelState, the current state of the vehicle
        #   target_point: [target_x, target_y]
        #   future_unreached_waypoints: a list of future waypoints[[target_x, target_y]]
        # Output: None

        curr_x, curr_y, curr_vel, curr_yaw = self.extract_vehicle_info(currentPose)

        # Acceleration Profile
        if self.log_acceleration:
            acceleration = (curr_vel- self.prev_vel) * 100 # Since we are running in 100Hz



        target_velocity = self.longititudal_controller(curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints)
        target_steering = self.pure_pursuit_lateral_controller(curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints)


        #Pack computed velocity and steering angle into Ackermann command
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = target_velocity
        newAckermannCmd.steering_angle = target_steering

        # Publish the computed control input to vehicle model
        self.controlPub.publish(newAckermannCmd)





        marker = Marker()
        marker.header.frame_id = "base_footprint"  # Use your desired frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "waypoints"
        marker.id = 0
        marker.type = Marker.POINTS  # Change to POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.2  # Size of the points
        marker.scale.y = 0.2  # Size of the points
        marker.color.a = 1.0  # Alpha (transparency)
        marker.color.r = 0.0  # Red
        marker.color.g = 1.0  # Green
        marker.color.b = 0.0  # Blue


        for wp in future_unreached_waypoints:
            point = Point()
            point.x = wp[0]
            point.y = wp[1]
            point.z = 0  # Assuming 2D
            marker.points.append(point)

        marker.header.stamp = rospy.Time.now()
        self.futurepoints.publish(marker)

    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = 0
        self.controlPub.publish(newAckermannCmd)
