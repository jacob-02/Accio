#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from functools import reduce
from laser_ranging.msg import laser
from laser_ranging.msg import obstacles

def Average(lst):
    return reduce(lambda a, b: a + b, lst) / len(lst)

pub = rospy.Publisher('results', laser, queue_size=10)
rospy.init_node('scan_data', anonymous=True)
rate = rospy.Rate(10)

def callback(msg):
    point = 0
    angle = 0
    distance = 0
    count = 0
    flag = 0
    prevPoint = point - 1
    laser_message = laser()
    obstacle_list = []
    obstacles_message = obstacles()

    for ranges in msg.ranges:
        if(ranges < 2):
            if((prevPoint+1) == point):
                if(count == 0):
                    obstacles_message.startAngle = angle
                distance = distance + ranges
                count = count + 1
                prevPoint = prevPoint + 1
                flag = 1

        else:
            prevPoint = point
            if(flag == 1):
                obstacles_message.avgDist = distance/count
                count = 0
                obstacles_message.endAngle = angle - 0.008714509196579456
                obstacle_list.append(obstacles_message)
                obstacles_message = obstacles()

                distance = 0
                flag = 0

        point = point + 1
        angle = angle + 0.008714509196579456
    laser_message.length = len(obstacle_list)
    laser_message.obstacles = obstacle_list

    # while not rospy.is_shutdown():
    # rospy.loginfo(laser_message)
    pub.publish(laser_message)

sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
