#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import PoseStamped

# Updates Unity with position of physical RC Cars on track from AprilTags
def unityCarUpdater():
	# initialize node
	rospy.init_node('unityCarUpdater', anonymous = True)

	#### Setup Car Position Publisher 
   	unityCarPublisher = rospy.Publisher("carPosition", PoseStamped, queue_size = 5)
	rate = rospy.Rate(0.1) # 0.1hz
	msg = PoseStamped()

        # Upper X and Y Bounds of the track
        upperXBound = 7
        upperYBound = 3

	while not rospy.is_shutdown():
	    # Initialize msg every loop with new coordinate point
	    msg.pose.position.x = random.uniform(0, upperXBound)
	    msg.pose.position.y = random.uniform(0, upperYBound)
	    
            # Modify rotation of car to face front of track
            msg.pose.orientation.w = 0.7071
	    msg.pose.orientation.x = 0
	    msg.pose.orientation.y = 0.7071
	    msg.pose.orientation.z = 0
		
	    # Publish msg
            rospy.loginfo(msg)
	    unityCarPublisher.publish(msg)
	    rate.sleep()

if __name__ == '__main__':
	unityCarUpdater()
