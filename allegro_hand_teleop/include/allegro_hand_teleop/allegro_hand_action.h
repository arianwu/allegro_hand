#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>

#include <sensor_msgs/JointState.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

enum eJointName
{
    eJOINTNAME_INDEX_0,
    eJOINTNAME_INDEX_1,
    eJOINTNAME_INDEX_2,
    eJOINTNAME_INDEX_3,
    eJOINTNAME_MIDDLE_0,
    eJOINTNAME_MIDDLE_1,
    eJOINTNAME_MIDDLE_2,
    eJOINTNAME_MIDDLE_3,
    eJOINTNAME_PINKY_0,
    eJOINTNAME_PINKY_1,
    eJOINTNAME_PINKY_2,
    eJOINTNAME_PINKY_3,
    eJOINTNAME_THUMB_0,
    eJOINTNAME_THUMB_1,
    eJOINTNAME_THUMB_2,
    eJOINTNAME_THUMB_3,
    ALLEGRO_DOF_JOINTS
};

const std::string JOINT_CMD_TOPIC = "joint_cmd";
const std::string JOINT_STATE_TOPIC = "joint_states";

// TIME STUFF
#include <time.h>

struct timespec start, stop;
float delta;

void start_timer();
void stop_timer();
void print_timer(std::string timer_name);

// END TIME STUFF

class AllegroHandActionNode {
public:
	AllegroHandActionNode();
	~AllegroHandActionNode();

	void jointCmdCallback(const sensor_msgs::JointState::ConstPtr& msg);
	void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
	void limitJoints();

private:
	ros::NodeHandle nh;

	ros::Subscriber joint_cmd_sub;
	ros::Subscriber joint_state_sub;

	sensor_msgs::JointState current_joint_state;

	bool ready;

	double joint_upper_limit[ALLEGRO_DOF_JOINTS] = {
		0.57, 1.71, 1.809, 1.718,  // Index Finger
		0.57, 1.71, 1.809, 1.718,  // Middle Finger
		0.57, 1.71, 1.809, 1.718,  // Pinky Finger
		1.4968, 1.13, 1.633, 1.81991  // Thumb
	};
	double joint_lower_limit[ALLEGRO_DOF_JOINTS] = {
		-0.57, -0.296, -0.274, -0.327,  // Index Finger
		-0.57, -0.296, -0.274, -0.327,  // Middle Finger
		-0.57, -0.296, -0.274, -0.327,  // Pinky Finger
		0.36357, -0.20504289, -0.2897, -0.2622  // Thumb
	};
	std::vector<double> joint_position = {
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0
	};
	std::vector<double> joint_velocity = {
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0
	};
	std::vector<std::string> joint_names = {
		"jif1", "jif2", "jif3", "jif4",
		"jmf1", "jmf2", "jmf3", "jmf4",
		"jpf1", "jpf2", "jpf3", "jpf4",
		"jth1", "jth2", "jth3", "jth4"
	};

	double exec_time;

	Client* robot_client;
	control_msgs::FollowJointTrajectoryGoal goal;
};