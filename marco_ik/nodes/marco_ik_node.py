#!/usr/bin/python
"""
ROS node for inverse kinematics of MARCO robots using numpy
"""

import rospy
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_matrix, rotation_matrix
from math import *

class IK_3R_Robot:


    def forward_kinematics(self):
        # Screw Axes
        w1 = np.array([[0, 0, 1]])
        w2 = np.array([[1, 0, 0]])
        w3 = np.array([[1, 0, 0]])
        w4 = np.array([[0, 0, 1]])
        w5 = np.array([[1, 0, 0]])
        w6 = np.array([[0, 0, 1]])
        w2 = rotation_matrix(self.q1, w1) * w2
        w2 = rotation_matrix(self.q1, w1) * w2
        self.T0 = np.eye(3)
        self.T1 = np.eye(3)

    def __init__(self):
        # Initialize node
        rospy.init_node('IK_3R_Robot', anonymous=True)
        # Create publisher
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        # Create subscriber
        self.sub = rospy.Subscriber('des_pose', Pose, self.cb_sub)
        self.q1, self.q2, self.q3, self.q4, self.q5, self.q6 = [0] * 6
        # Lengths
        self.l1, self.l2, self.l3, self.l4, self.l5, self.l6 = 0.13, 0.24, 0.33, 0.36, 0.23, 0.31
        self.timer = rospy.Timer(rospy.Duration(0.01), self.cb_tim)
        # IK Solutions (in radians)

    def cb_tim(self, event):
        js_msg = JointState()
        js_msg.header.frame_id = ''
        js_msg.header.stamp = rospy.get_rostime()
        js_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        js_msg.velocity = []
        js_msg.effort = []
        js_msg.position = [self.q1, self.q2, self.q3, 0, 0, 0]
        self.pub.publish(js_msg)

    def cb_sub(self, msg):
        """Inverse kinematics for a MARCO derived"""
        l1 = self.l1
        l2 = self.l2
        l3 = self.l3
        l4 = self.l4
        l5 = self.l5
        l6 = self.l6
        p6 = np.array([[msg.position.x, msg.position.y, msg.position.z]]).T
        R6 = quaternion_matrix([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        # Code BEGIN
        k6 = np.array([R6[:3,2]]).T
        p4 = p6 - (l5 + l6) * k6
        x6, y6, z6 = (p6[0], p6[1], p6[2])
        x4, y4, z4 = (p4[0], p4[1], p4[2])
        print(k6)
        acosarg = (x4 ** 2 + y4 ** 2 + (z4 - l1) ** 2 - l2 ** 2 - (l3 + l4) ** 2) / (2 * l2 * (l3 + l4))
        try:
            assert abs(acosarg) < 1
        except AssertionError as e: 
            return
        q3 = pi - acos(acosarg)
        alpha = atan2(z4 - l1, sqrt(x4 ** 2 + y4 ** 2))
        beta = atan2((l3 + l4) * sin(q3), l2 + (l3 + l4) * cos(q3))
        q2 = alpha + beta
        q1 = atan2(y4, x4)
        print(q1, q2, q3)
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3
        # Code END
        

if __name__ == '__main__':
    try:
        IK_3R_Robot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass