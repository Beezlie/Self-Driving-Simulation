#!/usr/bin/env python
import rospy
import random
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Vector3Stamped

# sends a waypoint to Unity for the car to drive to so it avoids obstacles
# gets obstacle coordinates from Unity and uses them to update waypoint 
def carController():
        obstacleDict = {}

	# initialize nodes
	rospy.init_node('carController', anonymous = True)

	# Setup waypoint Publisher 
   	waypointPublisher = rospy.Publisher("/car_control/waypoint", Vector3, queue_size = 5)
	rate = rospy.Rate(0.5) # 0.5hz
	waypointMsg = Vector3()

        # Setup obstacle subscribers
        rospy.Subscriber("/obstacle/position", Vector3Stamped, obstaclePositionCallback, (obstacleDict, ))
        rospy.Subscriber("/obstacle/destroy", String, obstacleDestroyedCallback, (obstacleDict, ))

        # Upper X and Y Bounds of the track
        upperXBound = 8.5
        upperYBound = 4.3
        lowerXBound = -1 * upperXBound
        lowerYBound = -1 * upperYBound

	while not rospy.is_shutdown():
	    # Initialize msg every loop with new waypoint
	    waypointMsg.x = random.uniform(lowerXBound + 3, upperXBound)

            obstacleY = []
            for obstacle in obstacleDict:
                obstacleY.append(obstacleDict[obstacle].y)
            obstacleY.sort()
            
            # Avoid y ranges that obstacles occupy
            possibleRanges = []
            lower = lowerYBound
            for ycoordinate in obstacleY:
                if (ycoordinate - lower) > 0.5:
                    possibleRanges.append((lower, ycoordinate - 0.5))
                lower = ycoordinate + 0.5

            if len(possibleRanges) > 0:
                rangeChoice = random.randint(0, len(possibleRanges) - 1)
                yRange = possibleRanges[rangeChoice]
                waypointMsg.y = random.uniform(yRange[0], yRange[1])
            else:
	        waypointMsg.y = random.uniform(0, upperYBound)
	    
            waypointMsg.z = 0
		
	    # Publish new waypoint
            rospy.loginfo(waypointMsg)
	    waypointPublisher.publish(waypointMsg)
	    rate.sleep()

def obstaclePositionCallback(data, args):
    print "Object updated"
    obstacleDict = args[0]
    obstacleID = data.header.frame_id
    x = data.vector.x
    y = data.vector.y
    z = data.vector.z

    if obstacleID in obstacleDict:
        rospy.loginfo(rospy.get_caller_id() + "Obstacle at (%f, %f, %f)", x, y, z)
    else:
        rospy.loginfo(rospy.get_caller_id() + "Obstacle created: %s", obstacleID)
    
    obstacleDict[obstacleID] = data.vector

def obstacleDestroyedCallback(data, args):
    print "object created"
    obstacleDict = args[0]
    obstacleID = data.data
    
    if obstacleID in obstacleDict:
        del obstacleDict[obstacleID]
        rospy.loginfo(rospy.get_caller_id() + "Obstacle destroyed: %s", obstacleID)

if __name__ == '__main__':
	carController()
