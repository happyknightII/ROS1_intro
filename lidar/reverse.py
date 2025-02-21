#!/usr/bin/env python3

# Import necessary ROS libraries
import math
import rospy  # Python client library for ROS
from geometry_msgs.msg import Twist  # Message type for controlling robot motion
from std_srvs.srv import Empty

#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import LaserScan
import numpy as np


def reset_simulation():
   rospy.wait_for_service('/reset')  # Wait until the reset service is available
   try:
       reset_turtle = rospy.ServiceProxy('/reset', Empty)  # Create a service proxy
       reset_turtle()  # Call the service
   except rospy.ServiceException as e:
       rospy.logerr("Service call failed: %s" % e)


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

    # Create a publisher that publishes to the /turtle1/cmd_vel topic with message type Twist
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Create a Twist message object to hold velocity commands
    move_cmd = Twist()

    # Set the linear velocity for forward movement
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0  # No rotation (angular velocity)

    # Set the linear velocity (in meters/second) and distance (in meters)
    distance = .3

    if min_front < distance:
        move_cmd.linear.x = -1.0
        pub.publish(move_cmd)
    pub.publish(move_cmd)

if __name__ == '__main__':
    try:
        rospy.init_node('global', anonymous=True)

        # Subscribe to the scan topic
        scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        # Handle exceptions and clean up if needed
        pass

