#!/usr/bin/env python
import time
import copy
import rospy
import actionlib
from sensor_msgs.msg import JointState
from control_msgs.msg import *
from trajectory_msgs.msg import *

# ---------------- TIMER ----------------
import time

start = 0.0
stop = 0.0
delta = 0.0

def start_timer():
    global start
    start = time.time()


def stop_timer():
    global stop, delta
    stop = time.time()
    delta = stop - start


def print_timer(name):
    print("DELTA TIME FOR {name_}: \t{delta_:.6f}".format(name_=name, delta_=delta))


start_timer()
stop_timer()
print_timer("test")

# ---------------- CLASS ----------------
class AllegroHandActionNode():
    def __init__(self):
        print('AllegroHandActionNode: initializing node')

        self.first_read = True
        self.ready = False
        self.joint_states = None

        self.exec_time = 0.0166;

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
        topic = 'joint_cmd'
        self.sub_joint_cmd = rospy.Subscriber(topic, JointState, self.joint_cmd_callback)
        topic = 'joint_states'
        self.sub_joint_states = rospy.Subscriber(topic, JointState, self.joint_states_callback)

        # Wait to get ready
        print('Waiting for joint_states...')
        while (not self.ready):
            pass
        print('Received first joint_states message')

        # Action Library for ros_control
        namespace = 'allegro_hand_controller'
        self.robot_client = actionlib.SimpleActionClient(namespace + '/follow_joint_trajectory', FollowJointTrajectoryAction)

        print('Waiting for server...')
        self.robot_client.wait_for_server()
        print('Connected to server')

        self.g = FollowJointTrajectoryGoal()
        self.g.trajectory = JointTrajectory()
        self.g.trajectory.joint_names = self.joint_names

        rospy.sleep(1)


    def joint_cmd_callback(self, msg):
        start_timer()
        # Extract data from message
        unlimited_joints = list(msg.position)

        # Limit position
        self.joint_position = self.limit_joints(unlimited_joints)

        # Send position
        self.robot_client.cancel_goal()
        self.g.trajectory.points = [JointTrajectoryPoint(positions=self.joint_position, velocities=self.joint_velocity, time_from_start=rospy.Duration(self.exec_time))]
        self.robot_client.send_goal(self.g)
        self.robot_client.wait_for_result()
        stop_timer()
        print_timer("TOTAL")


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
    print('starting allegro_hand_action')
    rospy.init_node('allegro_hand_action', disable_signals=True)
    node = AllegroHandActionNode()

    try:
        # node.main()
        rospy.spin()
    except:
        print('caught exception')
    print('exiting')