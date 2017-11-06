#!/usr/bin/env python
import roslib, rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point
import numpy as np
from robot_treasure_area.msg import RadiusMsg
import random
from robot_treasure_area.srv import ObjectChoice, ObjectChoiceResponse

treasure_location_x = 0.0
treasure_location_y = 0.0


def init():
    rospy.init_node('treasure_radius')

    # Works as a service provider 
    service = rospy.Service('gui_service', ObjectChoice, get_object)

    print 'Service Provider is listening ... '

    while not rospy.is_shutdown():
        rospy.spin()

"""
Callback function that will run when the service call is made. 
After getting the choice parameter, it specifies the treasure's location and makes the appropriate rosnode subscriptions/publications.
@request: <ObjectChoiceRequest>
@return: <ObjectChoiceResponse> True when the robot finds the treasure
"""
def get_object(request):
    global treasure_location_x, treasure_location_y, radiusPublisher

    try:
        trNum = request.choice

        amcl_topic = rospy.get_param('~amcl_topic','amcl_pose')

        treasure_location_x = rospy.get_param('~treasure_location'+ str(trNum) + '_x', 0.0)
        treasure_location_y = rospy.get_param('~treasure_location'+ str(trNum) + '_y', 0.0)
        radius_topic = rospy.get_param('~radius_topic','~treasure_radius')

        rospy.Subscriber(amcl_topic, PoseWithCovarianceStamped, treasure_distance_estimation)

        radiusPublisher = rospy.Publisher(radius_topic, RadiusMsg, queue_size=10)

        print 'You picked treasure ',trNum

        return ObjectChoiceResponse(True)
    except Exception ,e:
        print "Service provider failed: %s"%e
        return ObjectChoiceResponse(False)

"""
Publishes, through <radiusPublisher>, the distance between the robot's location and the specific treasure.
@pose_data: the data comming from the amcl_topic, that represent the pose of the robot at the specific time
"""
def treasure_distance_estimation(pose_data):
    global treasure_location_x, treasure_location_y, radiusPublisher

    poseData = pose_data.pose.pose

    robotPose = np.array([poseData.position.x, poseData.position.y])
    treasurePoint = np.array([treasure_location_x, treasure_location_y])

    # Euclidean distance between the robot's pose and the treasure's location
    radius = np.linalg.norm(robotPose-treasurePoint)

    radiusmsg = RadiusMsg() 
    radiusmsg.header.stamp = rospy.Time.now()
    radiusmsg.radius = radius
    radiusmsg.x = poseData.position.x   #position x of the robot
    radiusmsg.y = poseData.position.y   #position y of the robot
    radiusmsg.treasureX = treasure_location_x
    radiusmsg.treasureY = treasure_location_y

    radiusPublisher.publish(radiusmsg)


if __name__ == '__main__':
    init()