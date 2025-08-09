import rospy
from custom.msg import colour   

rospy.init_node('colour_publisher', anonymous=True)

rate = rospy.Rate(10)  # 10 Hz

pub = rospy.Publisher('/colour', colour, queue_size=10)

color = colour()
color.r = 255
color.g = 0
color.b = 0
color.a = 255
color.name = "White"

while not rospy.is_shutdown():
  
    pub.publish(color)
    rospy.loginfo("Publishing colour: %s", color.name)

    rate.sleep()