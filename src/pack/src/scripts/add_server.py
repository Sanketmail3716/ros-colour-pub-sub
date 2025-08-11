#!/usr/bin/env python3
import rospy
from custom.srv import add, addResponse   # Import both service type and response

def add_handler(req):
    result = req.a + req.b
    rospy.loginfo("Adding %d and %d to get %d", req.a, req.b, result)
    return addResponse(result)  # Must return a Response object

if __name__ == "__main__":
    rospy.init_node('add_server', anonymous=True)
    service = rospy.Service("add", add, add_handler)  # Capital S
    rospy.loginfo("Add service is ready.")
    rospy.spin()

"""   

if __name__ == "__main__":
    rospy.init_node('add_server', anonymous=True)
    service = rospy.Service("add", add, add_handler)  # Capital S
    rospy.loginfo("Add service is ready.")
    rospy.spin()

"""