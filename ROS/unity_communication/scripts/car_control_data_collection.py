#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64

def goalCallback(data):
    goal = data.pose
    rospy.loginfo(rospy.get_caller_id() + "Goal position: {}".format(goal.position))
    rospy.loginfo(rospy.get_caller_id() + "Goal rotation: {}".format(goal.quaternion))

def currentPoseCallback(data):
    pose = data.pose
    rospy.loginfo(rospy.get_caller_id() + "Current position: {}".format(pose.position))
    rospy.loginfo(rospy.get_caller_id() + "Current rotation: {}".format(pose.quaternion))

def currentTwistCallback(data):
    twist = data.twist
    rospy.loginfo(rospy.get_caller_id() + "Current twist linear: {}".format(twist.linear))
    rospy.loginfo(rospy.get_caller_id() + "Current twist angular: {}".format(twist.angular))

def throttleCallback(data):
    throttle = data.data
    rospy.loginfo(rospy.get_caller_id() + "Car Throttle: {}".format(throttle))

def steerCallback(data):
    steer = data.data
    rospy.loginfo(rospy.get_caller_id() + "Car Steer: {}".format(steer))

def trackSpeedCallback(data):
    speed = data.twist.linear.x
    rospy.loginfo(rospy.get_caller_id() + "Track speed: {}".format(speed))

def trackThrottleCallback(data):
    throttle = data.data
    rospy.loginfo(rospy.get_caller_id() + "Track throttle: {}".format(throttle))

def carControlDataListener():
    # Initialize node
    rospy.init_node('carControlDataListener', anonymous=True)

    # Subscribe to topics
    #TODO - need to create these topics in the source code if they don't exist yet
    rospy.Subscriber("/car/throttle", Float64, throttleCallback)
    rospy.Subscriber("/car/steer", Float64, steerCallback)
    rospy.Subscriber("nav/goal", PoseStamped, goalCallback)
    rospy.Subscriber("pose", PoseStamped, currentPoseCallback)
    rospy.Subscriber("twist", TwistStamped, currentTwistCallback)
    rospy.Subscriber("/treadmill/velocity", TwistStamped, trackSpeedCallback)
    rospy.Subscriber("/treadmill/throttle", Float64, trackThrottleCallback)
    rospy.spin()

if __name__ == '__main__':
    carControlDataListener()
