#!/usr/bin/env python3
import rospy
from custom.srv import add  # import service type

if __name__ == "__main__":
    rospy.init_node('add_client', anonymous=True)

    rospy.wait_for_service('add')  # Wait until service is available
    try:
        add_srv = rospy.ServiceProxy('add', add)  # create service proxy
        p, q = map(int, input("Enter two numbers to add (space-separated): ").split())
        resp = add_srv(p, q)  # call service
        rospy.loginfo("Result of adding %d and %d is %d", p, q, resp.sum)
        print("Result of adding {} and {} is {}".format(p, q, resp.sum))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
