#!/usr/bin/env python
import time
import roslib#; roslib.load_manifest('ur_driver')
import rospy
import actionlib
import numpy as np
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState

# ---------------- CLASS ----------------
class AllegroHandLoggerNode():
    def __init__(self):
        print("AllegroHandLoggerNode: initializing node")

        # Initialize Variables
        directory = "/home/ros/arian_ws/src/allegro_hand/allegro_hand_teleop/scripts/"
        self.goal_position_filename = directory + "logs/goal_position.txt"
        self.goal_velocity_filename = directory + "logs/goal_velocity.txt"
        self.goal_acceleration_filename = directory + "logs/goal_acceleration.txt"
        self.joint_states_position_filename = directory + "logs/joint_states_position.txt"
        self.joint_states_velocity_filename = directory + "logs/joint_states_velocity.txt"

        self.position = [0]*6
        self.velocity = [0]*6

        # Initialize files
        self.goal_position_file = open(self.goal_position_filename, 'w')
        self.goal_velocity_file = open(self.goal_velocity_filename, 'w')
        self.goal_acceleration_file = open(self.goal_acceleration_filename, 'w')
        self.joint_states_position_file = open(self.joint_states_position_filename, 'w')
        self.joint_states_velocity_file = open(self.joint_states_velocity_filename, 'w')

        # ROS Infrastructure
        topic = 'allegro_hand_controller/follow_joint_trajectory/goal'
        self.sub_goal = rospy.Subscriber(topic, FollowJointTrajectoryActionGoal, self.goal_callback)

        topic = 'joint_states'
        self.sub_joint_states = rospy.Subscriber(topic, JointState, self.joint_states_callback)


    def goal_callback(self, msg):
        # Extract points and start time from msg
        points = msg.goal.trajectory.points
        start_time = msg.header.stamp.to_sec()
        
        for i, point in enumerate(points):
            # Extract info from each point
            time_from_start = point.time_from_start.to_sec()
            position = np.array(point.positions)
            velocity = np.array(point.velocities)
            acceleration = np.array(point.accelerations)

            seconds = time_from_start + start_time

            # Write into respective file
            self.goal_position_file.write(str(seconds) + '\t' + '\t'.join(position.astype(str)) + '\n')
            self.goal_velocity_file.write(str(seconds) + '\t' + '\t'.join(velocity.astype(str)) + '\n')
            self.goal_acceleration_file.write(str(seconds) + '\t' + '\t'.join(acceleration.astype(str)) + '\n')


    def joint_states_callback(self, msg):
        # Extract information
        seconds = msg.header.stamp.to_sec()
        self.joint_state_position = np.array(msg.position)
        self.joint_state_velocity = np.array(msg.velocity)

        # Write into respective file
        self.joint_states_position_file.write(str(seconds) + '\t' + '\t'.join(self.joint_state_position.astype(str)) + '\n')
        self.joint_states_velocity_file.write(str(seconds) + '\t' + '\t'.join(self.joint_state_velocity.astype(str)) + '\n')


if __name__ == '__main__':
    print('starting allegro_hand_logger')
    rospy.init_node('allegro_hand_logger', disable_signals=True)
    node = AllegroHandLoggerNode()
    
    try:
        rospy.spin()
    except:
        node.close_files()
        print('caught exception')
    print('exiting')