import rospy 

from custom.msg import colour

rospy.init_node('colour_publisher', anonymous=True)

def cb(data):
    print("Received colour: R={}, G={}, B={}, A={}, Name={}".format(
        data.r, data.g, data.b, data.a, data.name))
    

rospy.Subscriber('/colour', colour, cb)

rospy.spin()
