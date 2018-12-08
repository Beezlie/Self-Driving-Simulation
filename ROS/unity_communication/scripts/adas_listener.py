#!/usr/bin/env python
import rospy
from unity_communication.msg import carPosition

def callback(data):
    carID = data.data.id
    rospy.loginfo(rospy.get_caller_id() + "Unity car %s next coordinate updated", carID)

# Receives coordinate decisions from Unity ML Agents
def adasListener():

    # Initialize node
    rospy.init_node('adasListener', anonymous=True)

    # Subscribe to topic to receive updates on number of obstacles on track
    rospy.Subscriber("adasPosition", carPosition, callback)
    rospy.spin()

if __name__ == '__main__':
    adasListener()
