#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "Treadmill speed: %f", data.data)
    
def trackListener():

    # Initialize node
    rospy.init_node('trackListener', anonymous=True)

    # Subscribe to topic to receive updates on track speed
    rospy.Subscriber("/treadmill/command_velocity", Float64, callback)
    rospy.spin()

if __name__ == '__main__':
    trackListener()
