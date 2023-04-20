#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import numpy as np
import time

# ---------------- CLASS ----------------
class AllegroHandPoserNode():
    def __init__(self):
        print('AllegroHandPoserNode: initializing node')

        # Initialize Variables
        self.joint_upper_limit = np.array([
            0.558488888889, 1.727825, 1.727825, 1.727825,  # Index Finger
            0.558488888889, 1.727825, 1.727825, 1.727825,  # Middle Finger
            0.558488888889, 1.727825, 1.727825, 1.727825,  # Pinky Finger
            1.57075, 1.15188333333, 1.727825, 1.76273055556  # Thumb
        ])
        self.joint_lower_limit = np.array([
            -0.558488888889, -0.279244444444, -0.279244444444, -0.279244444444,  # Index Finger
            -0.558488888889, -0.279244444444, -0.279244444444, -0.279244444444,  # Middle Finger
            -0.558488888889, -0.279244444444, -0.279244444444, -0.279244444444,  # Pinky Finger
            0.279244444444, -0.331602777778, -0.279244444444, -0.279244444444  # Thumb
        ])
        self.joint_names = [
            'jif1', 'jif2', 'jif3', 'jif4',
            'jmf1', 'jmf2', 'jmf3', 'jmf4',
            'jpf1', 'jpf2', 'jpf3', 'jpf4',
            'jth1', 'jth2', 'jth3', 'jth4'
        ]

        self.desired_joint_state = JointState(name=self.joint_names)

        self.file_number = 1
        self.name = "allegro_trajectories3"
        self.filename = "/home/ros/arian_ws/src/allegro_hand/allegro_hand_teleop/scripts/trajectories/" + self.name + "_" + str(self.file_number).zfill(2)
        self.freq = 20

        # ROS Infrastructure
        topic = "joint_cmd"
        self.joint_cmd_pub = rospy.Publisher(topic, JointState, queue_size=10)

        # Initialize array of commands (?)
        self.joint_commands_array = np.loadtxt(self.filename)

        assert self.joint_commands_array.shape[1] == 16

        # Call main
        self.main()


    def main(self):
        # Send first joint to ensure it is at starting position
        self.desired_joint_state.position = self.joint_commands_array[0]
        self.joint_cmd_pub.publish(self.desired_joint_state)
        self.joint_cmd_pub.publish(self.desired_joint_state)
        
        start_time = time.time()
        stop_time = time.time()
        while ((stop_time - start_time) < 3):
            stop_time = time.time()

        index = 0
        command_number = self.joint_commands_array.shape[0]
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown() and index < command_number:
            command = self.map_range(self.joint_commands_array[index])

            self.desired_joint_state.position = command

            self.joint_cmd_pub.publish(self.desired_joint_state)

            index += 1
            rate.sleep()
        print("Sequence finished")


    def map_range(self, x, in_min=-np.ones(16), in_max=np.ones(16)):
        out_min=self.joint_lower_limit
        out_max=self.joint_upper_limit
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


if __name__ == '__main__':
    print('starting allegro_hand_poser')
    rospy.init_node('allegro_hand_poser', disable_signals=True)
    node = AllegroHandPoserNode()

    try:
        # node.main()
        rospy.spin()
    except:
        print('caught exception')
    print('exiting')