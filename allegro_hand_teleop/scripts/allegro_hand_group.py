#!/usr/bin/env python
import time
import copy
import rospy
import actionlib
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# ---------------- CLASS ----------------
class AllegroHandGroupNode():
    def __init__(self):
        print('AllegroHandGroupNode: initializing node')
        
        self.ready = False
        self.joint_states = None

        # Initialize Variables
        self.joint_upper_limit = [
            0.57, 1.71, 1.809, 1.718,  # Index Finger
            0.57, 1.71, 1.809, 1.718,  # Middle Finger
            0.57, 1.71, 1.809, 1.718,  # Pinky Finger
            1.4968, 1.13, 1.633, 1.81991  # Thumb
        ]
        self.joint_lower_limit = [
            -0.57, -0.296, -0.274, -0.327,  # Index Finger
            -0.57, -0.296, -0.274, -0.327,  # Middle Finger
            -0.57, -0.296, -0.274, -0.327,  # Pinky Finger
            0.36357, -0.20504289, -0.2897, -0.2622  # Thumb
        ]
        self.joint_position = [
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0
        ]
        self.joint_velocity = [
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0
        ]
        self.joint_names = [
            'jif1', 'jif2', 'jif3', 'jif4',
            'jmf1', 'jmf2', 'jmf3', 'jmf4',
            'jpf1', 'jpf2', 'jpf3', 'jpf4',
            'jth1', 'jth2', 'jth3', 'jth4'
        ]

        self.command_message = Float64MultiArray()

        # ROS Infrastructure
        ns = 'allegro_hand_controller/'
        topic = 'joint_cmd'
        self.sub_joint_cmd = rospy.Subscriber(topic, JointState, self.joint_cmd_callback)
        topic = 'joint_states'
        self.sub_joint_states = rospy.Subscriber(topic, JointState, self.joint_states_callback)
        topic = 'command'
        self.pub_command = rospy.Publisher(ns + topic, Float64MultiArray, queue_size=10)

        # Wait to get ready
        print('Waiting for joint_states...')
        while (not self.ready):
            pass
        print('Received first joint_states message')


    def joint_cmd_callback(self, msg):
        # Extract data from message
        unlimited_joints = list(msg.position)

        # Limit position
        self.command_message.data = self.limit_joints(unlimited_joints)

        # Send position
        self.pub_command.publish(self.command_message)


    def joint_states_callback(self, msg):
        self.joint_states = msg
        self.ready = True


    def limit_joints(self, q):
        limited = copy.copy(q)
        for i in range(16):
            if q[i] > self.joint_upper_limit[i]:
                limited[i] = self.joint_upper_limit[i]
            elif q[i] < self.joint_lower_limit[i]:
                limited[i] = self.joint_lower_limit[i]
        return limited


if __name__ == '__main__':
    print('starting allegro_hand_group')
    rospy.init_node('allegro_hand_group', disable_signals=True)
    node = AllegroHandGroupNode()

    try:
        # node.main()
        rospy.spin()
    except:
        print('caught exception')
    print('exiting')