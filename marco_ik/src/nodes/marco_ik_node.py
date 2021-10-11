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
import tf
from math import *


def vec_to_so3(w_hat):
    w_hat = w_hat.reshape((3,))
    return np.array([[          0, -w_hat[2],  w_hat[1]], 
                     [ w_hat[2],           0, -w_hat[0]], 
                     [-w_hat[1],  w_hat[0],          0]])

class IK_3R_Robot:

    def __init__(self):
        # Initialize node
        rospy.init_node('IK_3R_Robot', anonymous=True)
        # Create publisher
        # Lengths
        self.l1, self.l2, self.l3, self.l4, self.l5, self.l6 = 0.13, 0.24, 0.33, 0.36, 0.23, 0.31
        self.timer = rospy.Timer(rospy.Duration(0.01), self.cb_tim)

        w1 = np.array([0, 0, 1])
        w2 = np.array([0, -1, 0])
        w3 = np.array([0, -1, 0])
        w4 = np.array([0, 0, 1])
        w5 = np.array([0, -1, 0])
        w6 = np.array([0, 0, 1])
        
        q1 = np.array([0, 0, 0])
        q2 = np.array([0, 0, self.l1])
        q3 = np.array([0, 0, self.l1 + self.l2])
        q4 = np.array([0, 0, self.l1 + self.l2 + self.l3])
        q5 = np.array([0, 0, self.l1 + self.l2 + self.l3 + self.l4])
        q6 = np.array([0, 0, self.l1 + self.l2 + self.l3 + self.l4 + self.l5])

        v1 = -vec_to_so3(w1) @ q1
        v2 = -vec_to_so3(w2) @ q2
        v3 = -vec_to_so3(w3) @ q3
        v4 = -vec_to_so3(w4) @ q4
        v5 = -vec_to_so3(w5) @ q5
        v6 = -vec_to_so3(w6) @ q6

        S1 = np.hstack((w1, v1))
        S2 = np.hstack((w2, v2))
        S3 = np.hstack((w3, v3))
        S4 = np.hstack((w4, v4))
        S5 = np.hstack((w5, v5))
        S6 = np.hstack((w6, v6))

        self.M = [np.eye(4)] * 7

        self.M[0] = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        self.M[1] = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, self.l1],
            [0, 0, 0, 1]
        ])
        self.M[2] = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, self.l1 + self.l2],
            [0, 0, 0, 1]
        ])
        self.M[3] = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, self.l1 + self.l2 + self.l3],
            [0, 0, 0, 1]
        ])
        self.M[4] = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, self.l1 + self.l2 + self.l3 + self.l4],
            [0, 0, 0, 1]
        ])
        self.M[5] = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, self.l1 + self.l2 + self.l3 + self.l4 + self.l5],
            [0, 0, 0, 1]
        ])
        self.M[6] = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, self.l1 + self.l2 + self.l3 + self.l4 + self.l5 + self.l6],
            [0, 0, 0, 1]
        ])

        self.Slist = np.array([S1, S2, S3, S4, S5, S6]).T


        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        # Create subscriber
        self.sub = rospy.Subscriber('/fk/out/pose', Pose, self.cb_sub)
        self.q = [0] * 6

        # IK Solutions (in radians)



    def get_transformation(self, i):
        w_hat = np.array([self.Slist[:3, i]]).T
        v_hat = np.array([self.Slist[3:, i]]).T
        theta = self.q[i]
        w_hat_mat = vec_to_so3(w_hat)
        R = np.eye(3) + sin(theta) * w_hat_mat + (1 - cos(theta)) * (w_hat_mat @ w_hat_mat)
        p = (np.eye(3) * theta + (1 - cos(theta)) * w_hat_mat + (theta - sin(theta)) * (w_hat_mat @ w_hat_mat)) @ v_hat
        T = np.vstack((np.hstack((R, p)), np.array([[0, 0, 0, 1]])))    
        return T

    def forward_kinematics(self, k):
        T = np.eye(4)
        for i in range(k):
            self.get_transformation(i)
            T = T @ self.get_transformation(i)
            
        T = T @ self.M[k]
        
        return T
        


    def cb_tim(self, event):
        js_msg = JointState()
        js_msg.header.frame_id = ''
        js_msg.header.stamp = rospy.get_rostime()
        js_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        js_msg.velocity = []
        js_msg.effort = []
        js_msg.position = [self.q[0], self.q[1], self.q[2], 0, 0, 0]
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
        k6 = np.array([R6[:3, 2]]).T
        p4 = p6 - (l5 + l6) * k6
        x6, y6, z6 = (p6[0], p6[1], p6[2])
        x4, y4, z4 = (p4[0], p4[1], p4[2])
        
        a = l2
        b = l3 + l4
        c = sqrt(x4 ** 2 + y4 ** 2 + (z4 - l1) ** 2)

        if a + b < c:
            return
        
        acosarg = (c ** 2 - a ** 2 - b ** 2) / (2 * a * b)

        q3 = pi - acos(acosarg)
        alpha = atan2(z4 - l1, sqrt(x4 ** 2 + y4 ** 2))
        beta = atan2((l3 + l4) * sin(q3), l2 + (l3 + l4) * cos(q3))
        q2 = alpha + beta
        q1 = atan2(y4, x4)
        self.q[0] = q1
        self.q[1] = q2
        self.q[2] = q3
        T3_test = self.forward_kinematics(3)
        

        # Code END
        

if __name__ == '__main__':
    try:
        IK_3R_Robot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass