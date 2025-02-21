#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
import sys

def publish_marker(x_goal, y_goal):
    rospy.init_node('marker_publisher')

    marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

    # Create a Marker object
    marker = Marker()
    marker.header.frame_id = "odom"  # Use "map" as the frame of reference
    marker.header.stamp = rospy.Time.now()
    marker.ns = "my_marker"
    marker.id = 0
    marker.type = Marker.SPHERE  # You can also use CUBE, ARROW, CYLINDER, etc.
    marker.action = Marker.ADD

    # Set the position at the provided (x_goal, y_goal) coordinates
    marker.pose.position.x = x_goal
    marker.pose.position.y = y_goal
    marker.pose.position.z = 0.0

    # Set the orientation (default orientation for a marker)
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # Set the scale of the marker (size of the marker)
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5

    # Set the color (in RGBA)
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0  # Make sure the alpha value is not 0

    # Publish the marker in a loop
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        marker_pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        # Check if enough arguments are passed
        if len(sys.argv) != 3:
            rospy.logerr("Usage: python3 <script_name.py> <x_goal> <y_goal>")
            sys.exit(1)

        # Take x and y values from command-line arguments
        x_goal = float(sys.argv[1])
        y_goal = float(sys.argv[2])
        
        publish_marker(x_goal, y_goal)
    except rospy.ROSInterruptException:
        pass
