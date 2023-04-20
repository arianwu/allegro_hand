#ifndef ALLEGRO_HAND_ROS_CONTROL_HW_INTERFACE
#define ALLEGRO_HAND_ROS_CONTROL_HW_INTERFACE

#include <ros/ros.h>
#include <pthread.h>
#include <time.h>
#include <math.h>
#include <string>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_interface.h>

#include <controller_manager/controller_manager.h>
#include <controller_manager_msgs/ListControllers.h>
#include "kdl_controller.h"
#include "allegro_hand_driver/AllegroHandDrv.h"
using namespace allegro;

// TIME STUFF
struct timespec start, stop;
float delta;

void start_timer();
void stop_timer();
void print_timer(std::string timer_name);

// END TIME STUFF

namespace allegro_hand_ros_control {

const std::string GRAV_ROT_TOPIC = "hand_gravity_vector";
const uint8_t window_size = 10;

class AllegroHandHWInterface : public hardware_interface::RobotHW {
public:
    AllegroHandHWInterface(const ros::NodeHandle &nh=ros::NodeHandle(""));
    ~AllegroHandHWInterface();

    void init_hw();
    void read(const ros::Time &time_now);
    void write(ros::Duration elapsed_time);

    void handGravityVectorCallback(const std_msgs::Float64MultiArray &msg);

protected:
    ros::NodeHandle n_;
    ros::Subscriber grav_rot_sub;

    joint_limits_interface::JointLimits limits;
    joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface_;

    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::EffortJointInterface jnt_effort_cmd_interface_;

    std::string jointNames[DOF_JOINTS] = {
        "jif1", "jif2", "jif3", "jif4",
        "jmf1", "jmf2", "jmf3", "jmf4",
        "jpf1", "jpf2", "jpf3", "jpf4",
        "jth1", "jth2", "jth3", "jth4"
    };

    double current_position[DOF_JOINTS] = {0.0};
    double previous_position[DOF_JOINTS] = {0.0};

    double current_position_filtered[DOF_JOINTS] = {0.0};
    double previous_position_filtered[DOF_JOINTS] = {0.0};

    double current_velocity[DOF_JOINTS] = {0.0};
    double previous_velocity[DOF_JOINTS] = {0.0};
    double current_velocity_filtered[DOF_JOINTS] = {0.0};
//public:
    double desired_torque[DOF_JOINTS] = {0.0};

    // ROS Time
    ros::Time tstart;
    ros::Time tnow;
    double dt;

    // CAN device
    allegro::AllegroHandDrv *canDevice;

    // Flags
    int lEmergencyStop = 0;

    // Robot Gravity Compensation
    allegroKDL* kdl_comp;
    std::vector<double> g_vec;
    Eigen::VectorXd tau_g = Eigen::VectorXd::Zero(DOF_JOINTS);
    Eigen::VectorXd current_position_eigen = Eigen::VectorXd::Zero(DOF_JOINTS);

    // NEW FILTER
    
    double velocity_sum[DOF_JOINTS] = {0.0};
    double velocity_filtered[DOF_JOINTS] = {0.0};
    double velocity_buffer[DOF_JOINTS][window_size] = {0.0};

};

} // end namespace allegro_hand_ros_control 

#endif //ALLEGRO_HAND_ROS_CONTROL_HW_INTERFACE