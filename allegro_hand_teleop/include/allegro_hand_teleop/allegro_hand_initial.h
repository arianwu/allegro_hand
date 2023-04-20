#include <ros/ros.h>
#include "ros/master.h"
#include <eigen3/Eigen/Eigen>
#include <algorithm>
#include <std_msgs/Float64MultiArray.h>
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

const std::string JOINT_STATE_TOPIC = "joint_states";
const std::string COMMAND_TOPIC = "allegro_hand_controller/command";

class AllegroHandInitialNode {
public:
	AllegroHandInitialNode();
	~AllegroHandInitialNode();

	void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
	void commandCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

private:
	ros::NodeHandle nh;

	ros::Subscriber joint_state_sub;
	ros::Publisher command_pub;

	sensor_msgs::JointState current_joint_state;
	std_msgs::Float64MultiArray command_msg;

	bool ready, finished;
};