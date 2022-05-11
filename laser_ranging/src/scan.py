#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from functools import reduce
from std_msgs.msg import Int16

point = 0
angle = 0
obstacles = []
startAngle = 0
endAngle = 0
distance = 0
avgDist = 0.0
count = 0
flag = 0
prevPoint = point - 1


def Average(lst):
    return reduce(lambda a, b: a + b, lst) / len(lst)


def talker():
    pub = rospy.Publisher('results', Int16, queue_size=10)
    # rospy.init_node('scan_data', anonymous=True)
    rate = rospy.Rate(10)
    global obstacles

    while not rospy.is_shutdown():
        # rospy.loginfo(obstacles)
        pub.publish(obstacles)
        rate.sleep()


def callback(msg):
    # print (len(msg.ranges))
    point = 0
    angle = 0
    obstacle = []
    global obstacles
    startAngle = 0
    endAngle = 0
    distance = 0
    avgDist = 0.0
    count = 0
    flag = 0
    prevPoint = point - 1

    for ranges in msg.ranges:
        if(ranges < 2):
            if((prevPoint+1) == point):
                if(count == 0):
                    startAngle = angle
                distance = distance + ranges
                count = count + 1
                prevPoint = prevPoint + 1
                flag = 1

        else:
            prevPoint = point
            if(flag == 1):
                avgDist = distance/count
                count = 0
                endAngle = angle - 0.008714509196579456
                # try:
                #     avgDist = Average(distance)

                # except:
                #     pass
                obstacle.append([avgDist, [startAngle, endAngle]])
                distance = 0
                flag = 0
            # print([ranges, point, angle])

        point = point + 1
        angle = angle + 0.008714509196579456
    # print([obstacles, len(obstacles)])
    # print()
    obstacles.append([obstacle, len(obstacle)])


rospy.init_node('scan_data')
sub = rospy.Subscriber('/scan', LaserScan, callback)
talker()
rospy.spin()
