#include "allegro_hand_teleop/allegro_hand_initial.h"

AllegroHandInitialNode::AllegroHandInitialNode(){
	// Initialize Variables
	ready = false;
	finished = false;

	// ROS Infrastructure
	joint_state_sub = nh.subscribe(JOINT_STATE_TOPIC, 1, &AllegroHandInitialNode::jointStateCallback, this);
	command_pub = nh.advertise<std_msgs::Float64MultiArray>(COMMAND_TOPIC, 5);

	// Wait to get ready
	ROS_INFO("Waiting for joint_states...");
	while(!ready)
		ros::spinOnce();
	ROS_INFO("Received first joint_states message");

	// Retrieve initial position
	command_msg.data = current_joint_state.position;

	std::vector<std::string> v;
	// Send initial goal
	while (ros::ok()){
		command_pub.publish(command_msg);
		ros::spinOnce();

		// ros::master::getNodes(v);
		// for (auto obj: v)
		// 	std::cout<<obj<<"\n";
		// std::cout<<std::endl;
		// if (std::find(v.begin(), v.end(), "/allegro_hand_hardware_interface") != v.end())
		// 	break;
	}
}

AllegroHandInitialNode::~AllegroHandInitialNode(){
	nh.shutdown();
}

void AllegroHandInitialNode::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
	current_joint_state = *msg;
	ready = true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "allegro_hand_initial");

	AllegroHandInitialNode node;
	//ros::spin();

	return 0;
}