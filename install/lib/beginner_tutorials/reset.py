#!/usr/bin/env python3

import rospy  # Python client library for ROS
from std_srvs.srv import Empty

def reset_simulation():
   rospy.wait_for_service('/reset')  # Wait until the reset service is available
   try:
       reset_turtle = rospy.ServiceProxy('/reset', Empty)  # Create a service proxy
       reset_turtle()  # Call the service
   except rospy.ServiceException as e:
       rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    try:
        reset_simulation()
    except rospy.ROSInterruptException:
        # Handle exceptions and clean up if needed
        pass