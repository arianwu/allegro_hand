#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# ---------------- CLASS ----------------
class AllegroHandInitialNode():
    def __init__(self):
        print('AllegroHandInitialNode: initializing node')

        # Initialize Variables
        self.ready = False
        self.joint_names = [
            'jif1', 'jif2', 'jif3', 'jif4',
            'jmf1', 'jmf2', 'jmf3', 'jmf4',
            'jpf1', 'jpf2', 'jpf3', 'jpf4',
            'jth1', 'jth2', 'jth3', 'jth4'
        ]
        self.joint_states = None

        # ROS Infrastructure
        topic = 'joint_states'
        self.sub_joint_states = rospy.Subscriber(topic, JointState, self.joint_states_callback)
        topic = "allegro_hand_controller/command"
        self.command_pub = rospy.Publisher(topic, Float64MultiArray, queue_size=1)

        # Wait to get ready
        print('Waiting for joint_states...')
        while (not self.ready):
            pass
        print('Received first joint_states message')

        msg = Float64MultiArray()
        msg.data = self.joint_states.position

        rate = rospy.Rate(1000) # 10 Hz
        while not rospy.is_shutdown():
            self.command_pub.publish(msg)
            rate.sleep()


    def joint_states_callback(self, msg):
        self.joint_states = msg
        self.ready = True



if __name__ == '__main__':
    print('starting allegro_hand_initial')
    rospy.init_node('allegro_hand_initial', disable_signals=True)
    node = AllegroHandInitialNode()

    try:
        pass
        #rospy.spin()
    except:
        print('caught exception')
    print('exiting')
