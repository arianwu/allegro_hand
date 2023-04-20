#include "allegro_hand_teleop/allegro_hand_action.h"

// TIME STUFF
void start_timer(){
	clock_gettime(CLOCK_REALTIME, &start);
}

void stop_timer(){
	clock_gettime(CLOCK_REALTIME, &stop);
}

void print_timer(std::string timer_name){
	delta = (stop.tv_sec + double(stop.tv_nsec)/1e+9) -  (start.tv_sec + double(start.tv_nsec)/1e+9);
	ROS_INFO("DELTA TIME FOR %s: \t%.6f", timer_name.c_str(), delta);
}

// END TIME STUFF

AllegroHandActionNode::AllegroHandActionNode(){
	// Initialize Variables
	ready = false;
	exec_time = 0.01;

	// ROS Infrastructure
	joint_cmd_sub = nh.subscribe(JOINT_CMD_TOPIC, 1, &AllegroHandActionNode::jointCmdCallback, this);
	joint_state_sub = nh.subscribe(JOINT_STATE_TOPIC, 1, &AllegroHandActionNode::jointStateCallback, this);

	// Wait to get ready
	ROS_INFO("Waiting for joint_states...");
	while(!ready)
		ros::spinOnce();
	ROS_INFO("Received first joint_states message");

	// Action Library for ros_control
	robot_client = new Client("allegro_hand_controller/follow_joint_trajectory", true);

	ROS_INFO("Waiting for server...");
	robot_client->waitForServer();
	ROS_INFO("Connected to server");

	goal.trajectory.joint_names = joint_names;
	goal.trajectory.points.resize(1);
	goal.trajectory.points[0].positions.resize(ALLEGRO_DOF_JOINTS);
	goal.trajectory.points[0].velocities.resize(ALLEGRO_DOF_JOINTS);
	goal.trajectory.points[0].velocities = joint_velocity;
}

AllegroHandActionNode::~AllegroHandActionNode(){
	delete robot_client;
	nh.shutdown();
}

void AllegroHandActionNode::jointCmdCallback(const sensor_msgs::JointState::ConstPtr& msg){
	start_timer();
	// Extract data from message
	joint_position = msg->position;

	// Limit position
	limitJoints();

	// Send goal
	goal.trajectory.points[0].positions = joint_position;
	goal.trajectory.points[0].time_from_start = ros::Duration(exec_time);
	//goal.trajectory.header.stamp = ros::Time::now();
	
	robot_client->cancelGoal();
	robot_client->sendGoal(goal);
	robot_client->waitForResult(ros::Duration(0.01));
	stop_timer();
	print_timer("TOTAL");
}

void AllegroHandActionNode::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
	current_joint_state = *msg;
	ready = true;
}

void AllegroHandActionNode::limitJoints(){
	for (size_t i=0; i<ALLEGRO_DOF_JOINTS; i++){
		if (joint_position[i] > joint_upper_limit[i])
			joint_position[i] = joint_upper_limit[i];
		else if (joint_position[i] < joint_lower_limit[i])
			joint_position[i] = joint_lower_limit[i];
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "allegro_hand_action");

	AllegroHandActionNode node;
	ros::spin();

	return 0;
}