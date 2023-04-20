#include "allegro_hand_teleop/allegro_hand_group.h"

AllegroHandGroupNode::AllegroHandGroupNode(){
	// Initialize Variables
	ready = false;
	
	// ROS Infrastructure
	joint_cmd_sub = nh.subscribe(JOINT_CMD_TOPIC, 1, &AllegroHandGroupNode::jointCmdCallback, this);
	joint_state_sub = nh.subscribe(JOINT_STATE_TOPIC, 1, &AllegroHandGroupNode::jointStateCallback, this);
	command_pub = nh.advertise<std_msgs::Float64MultiArray>(COMMAND_TOPIC, 5);
	
	// Wait to get ready
	ROS_INFO("Waiting for joint_states...");
	while(!ready)
		ros::spinOnce();
	ROS_INFO("Received first joint_states message");

	// Initialize position
	joint_position = current_joint_state.position;
}

AllegroHandGroupNode::~AllegroHandGroupNode(){
	nh.shutdown();
}

void AllegroHandGroupNode::jointCmdCallback(const sensor_msgs::JointState::ConstPtr& msg){
	// Extract data from message
	joint_position = msg->position;

	// Limit position
	limitJoints();

	// Send goal
	command_msg.data = joint_position;
	command_pub.publish(command_msg);
}

void AllegroHandGroupNode::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
	current_joint_state = *msg;
	ready = true;
}

void AllegroHandGroupNode::limitJoints(){
	for (size_t i=0; i<ALLEGRO_DOF_JOINTS; i++){
		if (joint_position[i] > joint_upper_limit[i])
			joint_position[i] = joint_upper_limit[i];
		else if (joint_position[i] < joint_lower_limit[i])
			joint_position[i] = joint_lower_limit[i];
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "allegro_hand_group");

	AllegroHandGroupNode node;
	ros::spin();

	return 0;
}