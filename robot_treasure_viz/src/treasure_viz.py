#!/usr/bin/env python
import roslib, rospy
from robot_treasure_area.msg import RadiusMsg
from visualization_msgs.msg import Marker

circleMarker = Marker()
frameId = ''
stateLimit = 1.5     #the limit (in meters) where the robot is "cold" or "hot" dependinmg on the distance from the treasure
extraRange = 0.2     #add an extra distance to the radius for a comfort zone (the treasure is surely into the visualized circle) - bigger coverage range in visualization

def init():
    global markerPublisher, frameId, treasurePublisher, stateLimit, extraRange

    rospy.init_node('treasure_marker')

    radius_topic = rospy.get_param('~radius_topic','~treasure_radius')
    rviz_marker_topic = rospy.get_param('~rviz_marker_topic', '~treasure_marker')
    frameId = rospy.get_param('~frame_id', 'map')
    stateLimit = rospy.get_param('~state_limit', stateLimit)
    extraRange = rospy.get_param('~extra_range', extraRange)

    rospy.Subscriber(radius_topic, RadiusMsg, treasure_area_viz)

    markerPublisher = rospy.Publisher(rviz_marker_topic, Marker, queue_size=5)
    treasurePublisher = rospy.Publisher("~treasure_location", Marker, queue_size=1)

    while not rospy.is_shutdown():
        rospy.spin()

def treasure_area_viz(radiusmsg):
    global circleMarker, markerPublisher, frameId, stateLimit, extraRange

    circleMarker.header.frame_id = frameId
    circleMarker.header.stamp = rospy.Time.now()
    circleMarker.type = Marker.SPHERE
    circleMarker.pose.position.x = radiusmsg.x
    circleMarker.pose.position.y = radiusmsg.y
    circleMarker.pose.position.z = 0

    circleMarker.pose.orientation.x = 0;
    circleMarker.pose.orientation.y = 0;
    circleMarker.pose.orientation.z = 0;
    circleMarker.pose.orientation.w = 1;

    circleMarker.scale.x = 2*(radiusmsg.radius + extraRange)
    circleMarker.scale.y = 2*(radiusmsg.radius + extraRange)
    circleMarker.scale.z = 0.1

    #if the robot is <stateLimit> meter away of the treasure -> the state goes from cold to hot!
    if radiusmsg.radius <= stateLimit:
        if reached_target(radiusmsg.radius):
            circleMarker.color.a = 0.8
            circleMarker.color.r = 0.0
            circleMarker.color.g = 1.0
            circleMarker.color.b = 0.0
        else:
            color = define_color('h', radiusmsg.radius)

            circleMarker.color.a = 0.8
            circleMarker.color.r = 1.0
            circleMarker.color.g = color
            circleMarker.color.b = color


    else:
        color = define_color('c', radiusmsg.radius)

        circleMarker.color.a = 0.8
        circleMarker.color.r = color
        circleMarker.color.g = color
        circleMarker.color.b = 1.0

    markerPublisher.publish(circleMarker)

    #Just for evaluation!!!
    #should not call this function wduring the game
    treasure_location_marker(radiusmsg.treasureX, radiusmsg.treasureY)
    
def reached_target(distance):
    global extraRange

    if distance <= extraRange:
        return True 

    return False

#Define the color based on the distance of the targer. 
#The closer it is, the more "red" it becomes. And via versa, the more away it goes t, the more "blue" it becomes
def define_color(state, distance):
    global stateLimit, extraRange

    #into the hot zone
    if state == 'h':
        x_max = stateLimit+extraRange
        x_min = 0.0

        normalize = (distance - x_min) / (x_max - x_min)

        return normalize
    #into the cold zone
    elif state == 'c':
        x_min = stateLimit+extraRange
        x_max = 2*stateLimit+extraRange

        if distance > x_max:
            return 0.0

        normalize = (distance - x_min) / (x_max - x_min)

        return (1.0 - normalize)
    else:
        return 0.5      #default


#Only for evaluation - mark the reasure
def treasure_location_marker(treasure_x, treasure_y):
    global frameId,treasurePublisher

    
    treasureMarker = Marker()

    treasureMarker.header.frame_id = frameId
    treasureMarker.header.stamp = rospy.Time.now()
    treasureMarker.type = Marker.SPHERE
    treasureMarker.pose.position.x = treasure_x
    treasureMarker.pose.position.y = treasure_y
    treasureMarker.pose.position.z = 0.2

    treasureMarker.pose.orientation.x = 0;
    treasureMarker.pose.orientation.y = 0;
    treasureMarker.pose.orientation.z = 0;
    treasureMarker.pose.orientation.w = 1;

    treasureMarker.scale.x = 0.2
    treasureMarker.scale.y = 0.2
    treasureMarker.scale.z = 0.2

    treasureMarker.color.a = 0.8
    treasureMarker.color.r = 0.9
    treasureMarker.color.g = 0.72
    treasureMarker.color.b = 0.0

    treasurePublisher.publish(treasureMarker)


if __name__ == '__main__':
    init()