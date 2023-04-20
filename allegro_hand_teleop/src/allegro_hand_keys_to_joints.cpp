#include "allegro_hand_teleop/allegro_hand_keys_to_joints.h"

AllegroHandKeysToJointNode::AllegroHandKeysToJointNode(){
	// Initialize Variables
	ready = false;
	joint_state_cmd.name = joint_names;

	// ROS Infrastructure
	joint_cmd_pub = nh.advertise<sensor_msgs::JointState>(JOINT_CMD_TOPIC, 5);
	keys_sub = nh.subscribe(KEYS_TOPIC, 1, &AllegroHandKeysToJointNode::keysCallback, this);
	joint_state_sub = nh.subscribe(JOINT_STATE_TOPIC, 1, &AllegroHandKeysToJointNode::jointStateCallback, this);

	// Wait to get ready
	ROS_INFO("Waiting for joint_states...");
	while(!ready)
		ros::spinOnce();
	ROS_INFO("Received first joint_states message");

	// Initialize position
	joint_position = current_joint_state.position;
}

AllegroHandKeysToJointNode::~AllegroHandKeysToJointNode(){
	nh.shutdown();
}

void AllegroHandKeysToJointNode::keysCallback(const std_msgs::String::ConstPtr& msg){
	if (!ready)
		return;
	
	double i = 0.05;
	std::string key = msg->data.c_str();
	char key_char = (char) key[0];

	switch (key_char) {
	case '1':
		finger = FINGER_THUMB;
		break;
	case '2':
		finger = FINGER_INDEX;
		break;
	case '3':
		finger = FINGER_MIDDLE;
		break;
	case '4':
		finger = FINGER_PINKY;
		break;

	case 'z':
		joint_position = {
			0, 0, 0, 0,
			0, 0, 0, 0,
			0, 0, 0, 0,
			0, 0, 0, 0
		};
		break;
	case 'm':
		joint_position = {
			-0.07, 1.08, 0.60, 0.62, 
			-0.03, 1.11, 0.62, 0.70, 
			-0.07, 1.06, 0.69, 0.75, 
			 1.50, 0.32, 0.30, 0.86
		};
		break;
	case 'x':
		joint_position = current_joint_state.position;
		break;

	case 'q':
		if (finger == FINGER_THUMB)
			joint_position[0+12] += i;
		else if (finger == FINGER_INDEX)
			joint_position[0+0] += i;
		else if (finger == FINGER_MIDDLE)
			joint_position[0+4] += i;
		else
			joint_position[0+8] += i;
		break;
	case 'a':
		if (finger == FINGER_THUMB)
			joint_position[0+12] -= i;
		else if (finger == FINGER_INDEX)
			joint_position[0+0] -= i;
		else if (finger == FINGER_MIDDLE)
			joint_position[0+4] -= i;
		else
			joint_position[0+8] -= i;
		break;
	case 'w':
		if (finger == FINGER_THUMB)
			joint_position[1+12] += i;
		else if (finger == FINGER_INDEX)
			joint_position[1+0] += i;
		else if (finger == FINGER_MIDDLE)
			joint_position[1+4] += i;
		else
			joint_position[1+8] += i;
		break;
	case 's':
		if (finger == FINGER_THUMB)
			joint_position[1+12] -= i;
		else if (finger == FINGER_INDEX)
			joint_position[1+0] -= i;
		else if (finger == FINGER_MIDDLE)
			joint_position[1+4] -= i;
		else
			joint_position[1+8] -= i;
		break;
	case 'e':
		if (finger == FINGER_THUMB)
			joint_position[2+12] += i;
		else if (finger == FINGER_INDEX)
			joint_position[2+0] += i;
		else if (finger == FINGER_MIDDLE)
			joint_position[2+4] += i;
		else
			joint_position[2+8] += i;
		break;
	case 'd':
		if (finger == FINGER_THUMB)
			joint_position[2+12] -= i;
		else if (finger == FINGER_INDEX)
			joint_position[2+0] -= i;
		else if (finger == FINGER_MIDDLE)
			joint_position[2+4] -= i;
		else
			joint_position[2+8] -= i;
		break;
	case 'r':
		if (finger == FINGER_THUMB)
			joint_position[3+12] += i;
		else if (finger == FINGER_INDEX)
			joint_position[3+0] += i;
		else if (finger == FINGER_MIDDLE)
			joint_position[3+4] += i;
		else
			joint_position[3+8] += i;
		break;
	case 'f':
		if (finger == FINGER_THUMB)
			joint_position[3+12] -= i;
		else if (finger == FINGER_INDEX)
			joint_position[3+0] -= i;
		else if (finger == FINGER_MIDDLE)
			joint_position[3+4] -= i;
		else
			joint_position[3+8] -= i;
		break;
	}

	// Limit position
	limitJoints();

	// Send message
	joint_state_cmd.header.stamp = ros::Time::now();
	joint_state_cmd.position = joint_position;
	joint_cmd_pub.publish(joint_state_cmd);
}

void AllegroHandKeysToJointNode::jointStateCallback(const sensor_msgs::JointState& msg){
	current_joint_state = msg;
	ready = true;
}

void AllegroHandKeysToJointNode::limitJoints(){
	for (size_t i=0; i<ALLEGRO_DOF_JOINTS; i++){
		if (joint_position[i] > joint_upper_limit[i])
			joint_position[i] = joint_upper_limit[i];
		else if (joint_position[i] < joint_lower_limit[i])
			joint_position[i] = joint_lower_limit[i];
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "allegro_hand_keys_to_joints");

	AllegroHandKeysToJointNode node;
	ros::spin();

	return 0;
}