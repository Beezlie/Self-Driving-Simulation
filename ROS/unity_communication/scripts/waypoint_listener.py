#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

def callback(data):
    pose = data.pose
    rospy.loginfo(rospy.get_caller_id() + "Waypoint position: {}".format(pose.position))
    rospy.loginfo(rospy.get_caller_id() + "Waypoint rotation: {}".format(pose.orientation))

# Receives coordinate decisions from Unity ML Agents
def waypointListener():

    # Initialize node
    rospy.init_node('waypointListener', anonymous=True)

    # Subscribe to topic to receive updates on waypoints
    rospy.Subscriber("nav/goal", PoseStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    waypointListener()

