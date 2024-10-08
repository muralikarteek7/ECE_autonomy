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


def quaternion_to_euler(x, y, z, w):
        # Convert quaternion to euler angles (roll, pitch, yaw)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

def calculate_curvature(x1, y1, x2, y2, x3, y3):
        # Calculate the curvature using three consecutive points
        # Points: (x1, y1), (x2, y2), (x3, y3)
        dx1 = x2 - x1
        dy1 = y2 - y1
        dx2 = x3 - x2
        dy2 = y3 - y2
        
        angle1 = np.arctan2(dy1, dx1)
        angle2 = np.arctan2(dy2, dx2)
        
        # The curvature is proportional to the change in angle
        curvature = np.abs(angle2 - angle1)
        
        return curvature

class vehicleController():

    def __init__(self):
        # Publisher to publish the control input to the vehicle model
        self.controlPub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size = 1)
        self.futurepoints = rospy.Publisher('future_waypoints', Marker, queue_size=10)
        self.prev_vel = 0
        self.L = 1.75 # Wheelbase, can be get from gem_control.py
        self.log_acceleration = False

        self.kp = 0
        self.ki = 0
        self.kd = 0
        self.preverror = 0
        self.sumerror = 0

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

        qx = currentPose.pose.orientation.x
        qy = currentPose.pose.orientation.y
        qz = currentPose.pose.orientation.z
        qw = currentPose.pose.orientation.w
        _, _, yaw = quaternion_to_euler(qx, qy, qz, qw)

        ####################### TODO: Your Task 1 code ends Here #######################

        return pos_x, pos_y, vel, yaw # note that yaw is in radian

    # Task 2: Longtitudal Controller
    # Based on all unreached waypoints, and your current vehicle state, decide your velocity
    
    


    def longititudal_controller(self, curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints):

        ####################### TODO: Your TASK 2 code starts Here #######################
        target_velocity = 10


        ####################### TODO: Your TASK 2 code ends Here #######################
        
        high_speed = 15.0  # for straight sections
        low_speed = 8.0   # for turns
        #print(curr_vel,len(future_unreached_waypoints))
        # If there are less than 3 waypoints, we can't calculate curvature
        if len(future_unreached_waypoints) < 3:
            return high_speed  # Default to high speed if insufficient waypoints
        
        # Extract the current and future waypoints
        #print(future_unreached_waypoints[0],future_unreached_waypoints[1])
        x1, y1 = future_unreached_waypoints[0]
        x2, y2 = future_unreached_waypoints[1]
        x3, y3 = future_unreached_waypoints[2]
        
        # Calculate the curvature based on the next 3 waypoints
        curvature = calculate_curvature(x1, y1, x2, y2, x3, y3)
        
        # Set target speed based on curvature
        if len(future_unreached_waypoints) > 94 or len(future_unreached_waypoints) < 12:
            target_vel = 17

        elif len(future_unreached_waypoints) < 32 and len(future_unreached_waypoints) > 24:
            target_vel = low_speed

        elif curvature < 0.2:  # Threshold for curvature, adjust as needed
            target_vel = high_speed  # Straight path
        else:
            target_vel = low_speed   # Shalen(future_unreached_waypoints) > 94rp turn
        
        return target_vel
        #return target_velocity


    # Task 3: Lateral Controller (Pure Pursuit)
    def pure_pursuit_lateral_controller(self, curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints):

        ####################### TODO: Your TASK 3 code starts Here #######################
        target_steering = 0

        ####################### TODO: Your TASK 3 code starts Here #######################

        #if len(future_unreached_waypoints) >
        if len(future_unreached_waypoints) > 38 or (len(future_unreached_waypoints) < 16 and len(future_unreached_waypoints) >2 ):
            lookahead_x, lookahead_y = target_point
            
            lookahead_x2, lookahead_y2 = future_unreached_waypoints[1]
        
            # Step 1: Calculate the lookahead distance (ld)
            ld = math.sqrt((lookahead_x - curr_x)**2 + (lookahead_y - curr_y)**2)
            ld2 = math.sqrt((lookahead_x2 - curr_x)**2 + (lookahead_y2 - curr_y)**2)
            ld = ld/2+ld2/2
            
            # Step 2: Compute the angle (α) between the vehicle's heading and the lookahead point
            angle_to_lookahead = math.atan2(lookahead_y - curr_y, lookahead_x - curr_x)
            angle_to_lookahead2 = math.atan2(lookahead_y2 - curr_y, lookahead_x2 - curr_x)
            alpha = angle_to_lookahead - curr_yaw
            alpha2 = angle_to_lookahead2 - curr_yaw

            alpha = alpha/2 + alpha2/2
            # Normalize alpha to be within [-pi, pi]
            alpha = (alpha + math.pi) % (2 * math.pi) - math.pi
        else:
            lookahead_x, lookahead_y = target_point
            
            #lookahead_x2, lookahead_y2 = future_unreached_waypoints[1]
        
            # Step 1: Calculate the lookahead distance (ld)
            ld = math.sqrt((lookahead_x - curr_x)**2 + (lookahead_y - curr_y)**2)
            #ld2 = math.sqrt((lookahead_x2 - curr_x)**2 + (lookahead_y2 - curr_y)**2)
            #ld = ld/2+ld2/2
            
            # Step 2: Compute the angle (α) between the vehicle's heading and the lookahead point
            angle_to_lookahead = math.atan2(lookahead_y - curr_y, lookahead_x - curr_x)
            #angle_to_lookahead2 = math.atan2(lookahead_y2 - curr_y, lookahead_x2 - curr_x)
            alpha = angle_to_lookahead - curr_yaw
            #alpha2 = angle_to_lookahead2 - curr_yaw

            #alpha = alpha/2 + alpha2/2
            # Normalize alpha to be within [-pi, pi]
            alpha = (alpha + math.pi) % (2 * math.pi) - math.pi

        print(alpha,target_point, len(future_unreached_waypoints))
        
        # Step 3: Calculate the steering angle (δ)
        if ld > 0:  # Avoid division by zero
            delta = math.atan((2 * self.L * math.sin(alpha)) / ld)
        else:
            delta = 0  # No steering if ld is zero (this scenario is unlikely)

        return delta
        
        
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
