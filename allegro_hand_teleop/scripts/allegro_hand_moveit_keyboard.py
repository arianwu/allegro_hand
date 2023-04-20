#!/usr/bin/env python
import sys
import time
import copy
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import JointState

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list

# ---------------- CLASS ----------------
class AllegroHandMoveItKeyboardNode():
    def __init__(self, commander):
        print('AllegroHandMoveItKeyboardNode: initializing node')

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
        self.finger = "thumb"

        # ROS Infrastructure
        topic = "keys"
        self.sub_keys = rospy.Subscriber(topic, String, self.keys_callback)

        # Initialize MoveIt Commander
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "allegro_hand"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        self.move_group.set_max_velocity_scaling_factor(1)
        self.move_group.set_max_acceleration_scaling_factor(1)

        rospy.sleep(1)


    def main(self):
        pass


    def keys_callback(self, msg):
        i = 0.1
        key = msg.data
        unlimited_joints = copy.copy(self.joint_position)

        if(key=='1'):
            self.finger = "thumb"
        elif(key=='2'):
            self.finger = "index"
        elif(key=='3'):
            self.finger = "middle"
        elif(key=='4'):
            self.finger = "pinky"
        elif(key=="z"):
            unlimited_joints = [
                0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0
            ]
        elif(key=="m"):
            unlimited_joints = [
                0.10, 1.60, 1.81, 1.72,
                4.45e-06, 1.21e-06, 3.30e-08, 2.62e-08,
                0.1, 1.40, 1.81, 1.72,
                1.46, 0.25, 0.89, 1.89
            ]

        if self.finger=="thumb":
            if(key=="q"):
                unlimited_joints[0+12] += i
            elif(key=="a"):
                unlimited_joints[0+12] -= i
            
            elif(key=="w"):
                unlimited_joints[1+12] += i
            elif(key=="s"):
                unlimited_joints[1+12] -= i
         
            elif(key=="e"):
                unlimited_joints[2+12] += i
            elif(key=="d"):
                unlimited_joints[2+12] -= i
         
            elif(key=="r"):
                unlimited_joints[3+12] += i
            elif(key=="f"):
                unlimited_joints[3+12] -= i
        elif self.finger=="index":
            if(key=="q"):
                unlimited_joints[0] += i
            elif(key=="a"):
                unlimited_joints[0] -= i
            
            elif(key=="w"):
                unlimited_joints[1] += i
            elif(key=="s"):
                unlimited_joints[1] -= i
         
            elif(key=="e"):
                unlimited_joints[2] += i
            elif(key=="d"):
                unlimited_joints[2] -= i
         
            elif(key=="r"):
                unlimited_joints[3] += i
            elif(key=="f"):
                unlimited_joints[3] -= i
        elif self.finger=="middle":
            if(key=="q"):
                unlimited_joints[0+4] += i
            elif(key=="a"):
                unlimited_joints[0+4] -= i
            
            elif(key=="w"):
                unlimited_joints[1+4] += i
            elif(key=="s"):
                unlimited_joints[1+4] -= i
         
            elif(key=="e"):
                unlimited_joints[2+4] += i
            elif(key=="d"):
                unlimited_joints[2+4] -= i
         
            elif(key=="r"):
                unlimited_joints[3+4] += i
            elif(key=="f"):
                unlimited_joints[3+4] -= i
        elif self.finger=="pinky":
            if(key=="q"):
                unlimited_joints[0+8] += i
            elif(key=="a"):
                unlimited_joints[0+8] -= i
            
            elif(key=="w"):
                unlimited_joints[1+8] += i
            elif(key=="s"):
                unlimited_joints[1+8] -= i
         
            elif(key=="e"):
                unlimited_joints[2+8] += i
            elif(key=="d"):
                unlimited_joints[2+8] -= i
         
            elif(key=="r"):
                unlimited_joints[3+8] += i
            elif(key=="f"):
                unlimited_joints[3+8] -= i

        # Limit position
        self.joint_position = self.limit_joints(unlimited_joints)
        print(self.joint_position)
        # Send position command through MoveIt
        self.move_group.go(self.joint_position)
        self.move_group.stop()


    def limit_joints(self, q):
        limited = copy.copy(q)
        for i in range(16):
            if q[i] > self.joint_upper_limit[i]:
                limited[i] = self.joint_upper_limit[i]
            elif q[i] < self.joint_lower_limit[i]:
                limited[i] = self.joint_lower_limit[i]
        return limited


if __name__ == '__main__':
    print('starting allegro_hand_moveit_keyboard')
    commander = moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('allegro_hand_moveit_keyboard', disable_signals=True)

    node = AllegroHandMoveItKeyboardNode(commander)

    try:
        rospy.spin()
    except:
        print('caught exception')
    print('exiting')