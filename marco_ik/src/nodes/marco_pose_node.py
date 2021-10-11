#!/usr/bin/python

"""
Write ROS node to publish Pose2D
"""

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point, Quaternion
from math import *

def talker():
    pub = rospy.Publisher('des_pose', Pose, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    i = 0
    while not rospy.is_shutdown():
        t = float(i) / 1000
        des_pos = Pose()
        des_pos.position = Point(0.0, 0.0, 1.2)
        des_pos.orientation = Quaternion(0, 0, 0, 1)
        pub.publish(des_pos)
        rate.sleep()
        i += 1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass