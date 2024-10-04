import time  # For tracking the current time

class vehicleController():

    def __init__(self):
        # Publisher to publish the control input to the vehicle model
        self.controlPub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size=1)
        self.futurepoints = rospy.Publisher('future_waypoints', Marker, queue_size=10)
        self.prev_vel = 0
        self.L = 1.75  # Wheelbase, can be obtained from gem_control.py
        self.log_acceleration = False

        # PID gains
        self.Kp_steering = 1.0  # Proportional gain
        self.Ki_steering = 0.1  # Integral gain
        self.Kd_steering = 0.05  # Derivative gain

        # PID state variables
        self.prev_steering_error = 0.0
        self.steering_integral = 0.0

        # Time tracking for dynamic dt
        self.prev_time = None

    def pure_pursuit_lateral_controller(self, curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints):
        # Get current time
        current_time = time.time()

        # Calculate dt based on the actual time elapsed
        if self.prev_time is None:
            dt = 0.1  # Default to 0.1s for the first iteration
        else:
            dt = current_time - self.prev_time

        # Update the previous time
        self.prev_time = current_time

        # Extract the next waypoint
        x, y = future_unreached_waypoints[0]
        if len(future_unreached_waypoints) > 1:
            x = (x + future_unreached_waypoints[1][0]) / 2
            y = (y + future_unreached_waypoints[1][1]) / 2

        # Compute the heading error (alpha)
        dy = y - curr_y
        dx = x - curr_x
        target_yaw = math.atan2(dy, dx)
        yaw_error = target_yaw - curr_yaw

        # Normalize the yaw error to the range [-pi, pi]
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        # Proportional term
        P_steering = self.Kp_steering * yaw_error

        # Integral term
        self.steering_integral += yaw_error * dt  # Use dynamic dt
        I_steering = self.Ki_steering * self.steering_integral

        # Derivative term
        D_steering = self.Kd_steering * (yaw_error - self.prev_steering_error) / dt  # Use dynamic dt
        self.prev_steering_error = yaw_error

        # Compute the steering angle using PID control
        target_steering = P_steering + I_steering + D_steering

        # Clamp the steering angle to the vehicle's steering limits (example: -0.5 to 0.5 rad)
        max_steering_angle = 0.5
        target_steering = max(-max_steering_angle, min(target_steering, max_steering_angle))

        return target_steering
