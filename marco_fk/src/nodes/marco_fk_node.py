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
import tf
from math import *

def vec_to_so3(w_hat):
    w_hat = w_hat.reshape((3,))
    return np.array([[          0, -w_hat[2],  w_hat[1]], 
                     [ w_hat[2],           0, -w_hat[0]], 
                     [-w_hat[1],  w_hat[0],          0]])

class FK_3R_Robot:

    def __init__(self):
        # Initialize node
        rospy.init_node('FK_3R_Robot', anonymous=True)
        # Create publisher
        self.pub = rospy.Publisher('/fk/out/pose', Pose, queue_size=10)
        # Create subscriber
        self.sub = rospy.Subscriber('/joint_states_des', JointState, self.cb_sub)
        self.pose = Pose()
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

        self.Msb = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, self.l1 + self.l2 + self.l3 + self.l4 + self.l5 + self.l6],
            [0, 0, 0, 1]
        ])

        self.Slist = np.array([S1, S2, S3, S4, S5, S6]).T
        

    def get_transformation(self, i):
        w_hat = np.array([self.Slist[:3, i]]).T
        v_hat = np.array([self.Slist[3:, i]]).T
        theta = self.theta[i]
        w_hat_mat = vec_to_so3(w_hat)
        R = np.eye(3) + sin(theta) * w_hat_mat + (1 - cos(theta)) * (w_hat_mat @ w_hat_mat)
        p = (np.eye(3) * theta + (1 - cos(theta)) * w_hat_mat + (theta - sin(theta)) * (w_hat_mat @ w_hat_mat)) @ v_hat
        T = np.vstack((np.hstack((R, p)), np.array([[0, 0, 0, 1]])))    
        return T

    def forward_kinematics(self):
        T = np.eye(4)
        for i in range(6):
            self.get_transformation(i)
            T = T @ self.get_transformation(i)
            
        T = T @ self.Msb
        p = T[:3, 3]
        
        q = tf.transformations.quaternion_from_matrix(T)
        
        self.pose.position.x = p[0]
        self.pose.position.y = p[1]
        self.pose.position.z = p[2]
        self.pose.orientation.x = q[0]
        self.pose.orientation.y = q[1]
        self.pose.orientation.z = q[2]
        self.pose.orientation.w = q[3]
        

    def cb_tim(self, event):
        self.pub.publish(self.pose)

    def cb_sub(self, msg):
        """Forward kinematics for a MARCO derived"""
        l1 = self.l1
        l2 = self.l2
        l3 = self.l3
        l4 = self.l4
        l5 = self.l5
        l6 = self.l6
        self.theta = [0] * 6
        self.theta[0] = msg.position[0]
        self.theta[1] = msg.position[1]
        self.theta[2] = msg.position[2]
        self.theta[3] = msg.position[3]
        self.theta[4] = msg.position[4]
        self.theta[5] = msg.position[5]
        
        self.forward_kinematics()
        

if __name__ == '__main__':
    try:
        FK_3R_Robot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass