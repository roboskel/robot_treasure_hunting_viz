#!/usr/bin/env python
import roslib, rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point
import numpy as np
from robo_treasure.msg import RadiusMsg

treasure_location_x = 10.0
treasure_location_y = 0.0


def init():
    global treasure_location_x, treasure_location_y, radiusPublisher

    rospy.init_node('treasure_radius')

    amcl_topic = rospy.get_param('~amcl_topic','amcl_pose')
    treasure_location_x = rospy.get_param('~treasure_location_x', 0.0)
    treasure_location_y = rospy.get_param('~treasure_location_y', 0.0)
    radius_topic = rospy.get_param('~radius_topic','~treasure_radius')

    rospy.Subscriber(amcl_topic, PoseWithCovarianceStamped, treasure_distance_estimation)

    radiusPublisher = rospy.Publisher(radius_topic, RadiusMsg, queue_size=10)

    while not rospy.is_shutdown():
        rospy.spin()


def treasure_distance_estimation(pose_data):
    global treasure_location_x, treasure_location_y, radiusPublisher

    poseData = pose_data.pose.pose

    robotPose = np.array([poseData.position.x, poseData.position.y])
    treasurePoint = np.array([treasure_location_x, treasure_location_y])


    #euclidean distance between the robot's pose and the treasure's location
    radius = np.linalg.norm(robotPose-treasurePoint)

    radiusmsg = RadiusMsg() 
    radiusmsg.header.stamp = rospy.Time.now()
    radiusmsg.radius = radius
    radiusmsg.x = poseData.position.x
    radiusmsg.y = poseData.position.y

    radiusPublisher.publish(radiusmsg)


if __name__ == '__main__':
    init()