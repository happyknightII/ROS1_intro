#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class PathPublisher:
    def __init__(self):
        rospy.init_node('path_publisher', anonymous=True)
        self.path_pub = rospy.Publisher('/robot_path', Path, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.path = Path()
        self.path.header.frame_id = "odom"  # Set the fixed frame as 'odom'
        self.reset_path = False  # Flag to reset the path
        rospy.on_shutdown(self.cleanup)  # Register shutdown function

    def odom_callback(self, data):
        # Reset the path if reset flag is true
        if self.reset_path:
            self.path.poses = []
            self.reset_path = False  # Reset the flag after clearing the path

        # Create a PoseStamped object to store the robot's current position
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.header.stamp = rospy.Time.now()

        # Copy the position from the Odometry message
        pose.pose.position = data.pose.pose.position

        # Copy the orientation from the Odometry message
        pose.pose.orientation = data.pose.pose.orientation

        # Append the current pose to the path
        self.path.poses.append(pose)

        # Update the Path message header with the current time
        self.path.header.stamp = rospy.Time.now()

        # Publish the path
        self.path_pub.publish(self.path)

    def reset(self):
        rospy.loginfo("Path reset requested")
        self.reset_path = True  # Set the flag to reset the path

    def cleanup(self):
        rospy.loginfo("Shutting down, clearing path")
        # Clear the path before shutting down
        self.path.poses = []
        self.path_pub.publish(self.path)  # Publish an empty path to clear it in RViz

    def publish(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        path_publisher = PathPublisher()
        path_publisher.publish()
    except rospy.ROSInterruptException:
        pass
