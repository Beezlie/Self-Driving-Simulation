#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

def demo():
    # initialize node
    rospy.init_node('demo', anonymous = True)

    #### Setup car position Publishers
    carPosPublisher = rospy.Publisher("/car/0/pose", PoseStamped, queue_size = 5)
    rate = rospy.Rate(0.1) # 0.1hz
    posMsg = PoseStamped()

    #### Setup track velocity Publisher
    treadmillPublisher = rospy.Publisher("/treadmill/velocity", Float64, queue_size = 5)
    rate = rospy.Rate(0.1) # 0.1hz
    velMsg = Float64()

    while not rospy.is_shutdown():
        # Initialize msg every loop with new coordinate point
        posMsg.pose.position.z = random.uniform(0, 0)  #z=y
        posMsg.pose.position.x = random.uniform(0, 20)   #x=z
        posMsg.pose.position.y = random.uniform(0, 10)  #y=x

        posMsg.pose.orientation.w = 1
        posMsg.pose.orientation.x = 0
        posMsg.pose.orientation.y = 0
        posMsg.pose.orientation.z = 0

        # Publish car position msg
        rospy.loginfo(posMsg)
        carPosPublisher.publish(posMsg)

        # Publish track velocity msg
        velMsg.data = 1
        rospy.loginfo(velMsg)
        treadmillPublisher.publish(velMsg)

        rate.sleep()

if __name__ == '__main__':
    demo()
