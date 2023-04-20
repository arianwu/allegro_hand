#!/usr/bin/env python
import time
import copy
import rospy
import actionlib
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from control_msgs.msg import *
from trajectory_msgs.msg import *

# ---------------- CLASS ----------------
class AllegroHandActionKeyboardNode():
    def __init__(self):
        print('AllegroHandActionKeyboardNode: initializing node')

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
        self.finger = "thumb"

        self.exec_time = 1

        # ROS Infrastructure
        topic = 'keys'
        self.sub_keys = rospy.Subscriber(topic, String, self.keys_callback)
        topic = 'joint_states'
        self.sub_joint_states = rospy.Subscriber(topic, JointState, self.joint_states_callback)
        
        # Wait to get ready
        print('Waiting for joint_states...')
        while (not self.ready):
            pass
        print('Received first joint_states message')
        
        self.joint_position = list(self.joint_states.position)

        # Action Library for ros_control
        namespace = 'allegro_hand_controller'
        self.robot_client = actionlib.SimpleActionClient(namespace + '/follow_joint_trajectory', FollowJointTrajectoryAction)

        print('Waiting for server...')
        self.robot_client.wait_for_server()
        print('Connected to server')

        self.g = FollowJointTrajectoryGoal()
        self.g.trajectory = JointTrajectory()
        self.g.trajectory.joint_names = self.joint_names

        # Initial Position (Optional)
        # self.g.trajectory.points = [JointTrajectoryPoint(positions=self.joint_position, velocities=self.joint_velocity, time_from_start=rospy.Duration(2.0))]
        # self.robot_client.send_goal(self.g)
        # self.robot_client.wait_for_result()
        rospy.sleep(1)


    def keys_callback(self, msg):
        if not self.ready:
            return

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
        elif(key=="x"):
            unlimited_joints = list(self.joint_states.position)

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

        # Send position
        self.robot_client.cancel_goal()
        self.g.trajectory.points = [JointTrajectoryPoint(positions=self.joint_position, velocities=self.joint_velocity, time_from_start=rospy.Duration(0.1))]
        self.robot_client.send_goal(self.g)
        self.robot_client.wait_for_result()


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
    print('starting allegro_hand_action_keyboard')
    rospy.init_node('allegro_hand_action_keyboard', disable_signals=True)
    node = AllegroHandActionKeyboardNode()

    try:
        rospy.spin()
    except:
        print('caught exception')
    print('exiting')