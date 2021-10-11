#!/usr/bin/python
"""
ROS node for inverse kinematics of MARCO robots using numpy
"""

import rospy
from sensor_msgs.msg import JointState
import numpy as np
from math import *        

if __name__ == '__main__':
    try:
        rospy.init_node('fk_demo', anonymous=True)
        pub = rospy.Publisher('/joint_states_des', JointState, queue_size=10)
        i = 0
        while not rospy.is_shutdown():
            js_msg = JointState()
            js_msg.header.frame_id = ''
            js_msg.header.stamp = rospy.get_rostime()
            js_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
            js_msg.position = [0] * 6
            js_msg.velocity = []
            js_msg.effort = []
            js_msg.position[0] = -0.3
            js_msg.position[1] = -pi / 3
            js_msg.position[2] = -pi / 2
            js_msg.position[3] = 0.123
            js_msg.position[4] = -1.2
            js_msg.position[5] = 0.5
            pub.publish(js_msg)
            rospy.sleep(0.1)
            i += 1
    except rospy.ROSInterruptException:
        pass