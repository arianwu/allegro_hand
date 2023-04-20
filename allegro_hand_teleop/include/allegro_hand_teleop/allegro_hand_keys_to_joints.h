#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>

#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

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

enum allegroFingers
{
	FINGER_THUMB,
	FINGER_INDEX,
	FINGER_MIDDLE,
	FINGER_PINKY
};

const std::string KEYS_TOPIC = "keys";
const std::string JOINT_CMD_TOPIC = "joint_cmd";
const std::string JOINT_STATE_TOPIC = "joint_states";

class AllegroHandKeysToJointNode {
public:
	AllegroHandKeysToJointNode();
	~AllegroHandKeysToJointNode();

	void keysCallback(const std_msgs::String::ConstPtr& msg);
	void jointStateCallback(const sensor_msgs::JointState& msg);
	void limitJoints();

private:
	ros::NodeHandle nh;

	ros::Subscriber keys_sub;
	ros::Publisher joint_cmd_pub;
	ros::Subscriber joint_state_sub;

	sensor_msgs::JointState joint_state_cmd;
	sensor_msgs::JointState current_joint_state;

	bool ready;
	int finger = FINGER_THUMB;

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
};