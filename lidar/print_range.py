#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import LaserScan
import numpy as np

def scan_callback(msg):
    # Print the total number of scan elements (360 for a full 360-degree LIDAR scan)
    num_elements = len(msg.ranges)
    print(f"Total scan elements: {num_elements}")

    # Print the range directly ahead of the robot
    range_ahead = msg.ranges[0]
    print(f"Range ahead: {range_ahead:.2f} meters")

    # TODO: Uncomment and complete the code to divide the scan data into front, left, and right sections
    # Hint: can use list comprehensions
    front = msg.ranges[:60] + msg.ranges[300:]
    left = msg.ranges[120:240]
    right = msg.ranges[60:120]

    # TODO: Uncomment and complete the code to find the minimum distance in each section
    min_front = min(front)
    min_left = min(left)
    min_right = min(right)

    # TODO: Print the min distance from the front, left, and right sections
    print(f"Min distance ahead: {min_front:.2f} meters")
    print(f"Min distance to the left: {min_left:.2f} meters")
    print(f"Min distance to the right: {min_right:.2f} meters")

# Initialize the node
rospy.init_node('lidar_scan')

# Subscribe to the scan topic
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)

# Keep the program running
rospy.spin()
