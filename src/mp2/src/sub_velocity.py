#!/usr/bin/env python

import rospy
import csv
from std_msgs.msg import Float32MultiArray

# Global variable to hold the CSV file object
csv_file = None
csv_writer = None

def callback(data):
    global csv_writer

    # Check if the data received has the expected number of elements
    if len(data.data) < 5:
        rospy.logwarn("Received insufficient data")
        return

    # Extract x, y, and other parameters from the received data
    x = data.data[0]
    y = data.data[1]
    v = data.data[2]
    yaw = data.data[3]
    acceleration = data.data[4]
    timet = data.data[5]

    # Log the received data
    rospy.loginfo(f"Received data: x={x}, y={y}, v={v}, yaw={yaw}, acceleration={acceleration}, time = {timet}")

    # Write the data to the CSV file
    csv_writer.writerow([x, y, v, yaw, acceleration, timet])
    csv_file.flush()  # Ensure data is written to disk

def subscribe_vehicle_data():
    global csv_file, csv_writer

    # Initialize the ROS node
    rospy.init_node('vehicle_data_subscriber', anonymous=True)

    # Get the topic name from parameters, with a default
    topic_name = rospy.get_param('~vehicle_data_topic', 'vehicle_data')
    
    # Subscribe to the vehicle_data topic
    rospy.Subscriber(topic_name, Float32MultiArray, callback)
    
    # Open a CSV file for writing
    csv_file = open('vehicle_data.csv', mode='w', newline='')
    csv_writer = csv.writer(csv_file)
    
    # Write the header to the CSV file
    csv_writer.writerow(['x', 'y', 'velocity', 'yaw', 'acceleration','time'])

    # Keep the node running
    rospy.spin()

    # Close the CSV file when done
    csv_file.close()

if __name__ == '__main__':
    try:
        subscribe_vehicle_data()
    except rospy.ROSInterruptException:
        pass
