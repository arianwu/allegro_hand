#include "allegro_hand_ros_control/allegro_hand_hardware_interface.h"

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

namespace allegro_hand_ros_control {

AllegroHandHWInterface::AllegroHandHWInterface(const ros::NodeHandle &nh):n_(nh){
    // Initialize CAN Device
    canDevice = 0;
    canDevice = new allegro::AllegroHandDrv();
    if (canDevice->init()) {
        usleep(3000);
    }
    else {
        delete canDevice;
        canDevice = 0;
    }

    // Initialize kdl
    g_vec = std::vector<double>(3);
    kdl_comp = new allegroKDL(g_vec, 0);

    // Start ROS timer
    tstart = ros::Time::now();

    // ROS Infrastructure
    grav_rot_sub = n_.subscribe(GRAV_ROT_TOPIC, 1, //300, // queue size
                                &AllegroHandHWInterface::handGravityVectorCallback, this);

    // TESTING LOOP
    // {
    //     Eigen::Vector3d xd;
    //     ros::Rate r(10);
    //     while (ros::ok()) {
    //         break;
    //         // Update Gravity Vector
    //         kdl_comp->update_G(g_vec);

    //         // Update current_position_eigen
    //         for (int iterator=0; iterator < DOF_JOINTS; iterator++) {
    //             current_position_eigen[iterator] = current_position_filtered[iterator];
    //         }

    //         // Get Gravity torques
    //         kdl_comp->get_G(current_position_eigen, tau_g);

    //         xd = Eigen::Vector3d(g_vec.data());
    //         std::cout<<tau_g<<std::endl<<std::endl;
    //         ros::spinOnce();
    //         r.sleep();
    //     }
    // }

    init_hw();
}

AllegroHandHWInterface::~AllegroHandHWInterface(){}

void AllegroHandHWInterface::init_hw(){
    boost::shared_ptr<urdf::ModelInterface> urdf(new urdf::ModelInterface);

    for(size_t i=0; i<DOF_JOINTS; i++){
        // Create joint_state_interface
        hardware_interface::JointStateHandle jointStateHandleTmp(jointNames[i], &current_position_filtered[i], &current_velocity_filtered[i], &desired_torque[i]);
        jnt_state_interface_.registerHandle(jointStateHandleTmp);

        // Create effort joint interface
        hardware_interface::JointHandle jointEffortHandleTmp(jointStateHandleTmp, &desired_torque[i]);
        jnt_effort_cmd_interface_.registerHandle(jointEffortHandleTmp);

        // Create Joint Limit (TODO)
        urdf::JointConstSharedPtr urdf_joint = urdf->getJoint(jointNames[i]);
        const bool urdf_limits_ok = joint_limits_interface::getJointLimits(urdf_joint, limits);
        joint_limits_interface::PositionJointSaturationHandle jointLimitsHandleTmp(jointEffortHandleTmp, limits);
        position_joint_saturation_interface_.registerHandle(jointLimitsHandleTmp);

        //joint_limits_interface::getJointLimits(jointNames[i], n_, limits);

    }
    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_effort_cmd_interface_);
    registerInterface(&position_joint_saturation_interface_); 
}

void AllegroHandHWInterface::read(const ros::Time &time_now){
    // Calculate loop time;
    tnow = ros::Time::now();

    dt = 1e-9 * (tnow - tstart).nsec;

    // When running gazebo, sometimes the loop gets called *too* often and dt will
    // be zero. Ensure nothing bad (like divide-by-zero) happens because of this.
    if(dt <= 0) {
        ROS_DEBUG_STREAM_THROTTLE(1, "AllegroNode::updateController dt is zero.");
        return;
    }

    tstart = tnow;

    if(canDevice){
        // try to update joint positions through CAN comm:
        lEmergencyStop = canDevice->readCANFrames();

        if(lEmergencyStop == 0 && canDevice->isJointInfoReady()){
            // back-up previous joint positions:
            for (int i = 0; i < DOF_JOINTS; i++) {
                previous_position[i] = current_position[i];
                previous_position_filtered[i] = current_position_filtered[i];
                previous_velocity[i] = current_velocity[i];
            }
            // update joint positions:
            canDevice->getJointInfo(current_position);
        }
        // NEW FILTER
        for (int joint = 0; joint < DOF_JOINTS; joint++) {
            velocity_sum[joint] = velocity_sum[joint] - velocity_buffer[joint][0] + current_velocity[joint];
            for (int i=0; i < window_size-1; i++){
                velocity_buffer[joint][i] = velocity_buffer[joint][i+1];
            }
            velocity_buffer[joint][window_size-1] = current_velocity[joint];
            velocity_filtered[joint] = velocity_sum[joint] / window_size;
        }

        // low-pass filtering:
        for (int i = 0; i < DOF_JOINTS; i++) {
            current_position_filtered[i] = (0.6 * current_position_filtered[i]) +
                                           (0.2 * previous_position[i]) +
                                           (0.2 * current_position[i]);
            current_velocity[i] =
                    (current_position_filtered[i] - previous_position_filtered[i]) / dt;
            current_velocity_filtered[i] = (0.6 * current_velocity_filtered[i]) +
                                           (0.2 * previous_velocity[i]) +
                                           (0.2 * current_velocity[i]);
            current_velocity[i] = (current_position[i] - previous_position[i]) / dt;

            current_position_eigen[i] = current_position_filtered[i];
        }

        // Obtaining the gravity compensation torques using the dynamic gravity vector
        kdl_comp->update_G(g_vec);
        kdl_comp->get_G(current_position_eigen, tau_g);
    }
}

void AllegroHandHWInterface::write(ros::Duration elapsed_time){
    // Add gravity compensation
    for (size_t i=0; i<DOF_JOINTS; i++)
        desired_torque[i] += tau_g[i];

    // Safety
    position_joint_saturation_interface_.enforceLimits(elapsed_time);

    // set & write torque to each joint:
    canDevice->setTorque(desired_torque);
    lEmergencyStop = canDevice->writeJointTorque();

	// reset joint position update flag:
    canDevice->resetJointInfoReady();
}

void AllegroHandHWInterface::handGravityVectorCallback(const std_msgs::Float64MultiArray &msg){
    g_vec[0] = msg.data[0];
    g_vec[1] = msg.data[1];
    g_vec[2] = msg.data[2];
}

} //end namespace allegro_hand_ros_control


typedef struct{
    controller_manager::ControllerManager *manager;
    allegro_hand_ros_control::AllegroHandHWInterface *allegro_hand_hw_interface;
}ArgsForThread;

static void timespecInc(struct timespec &tick, int nsec)
{
    int SEC_2_NSEC = 1e+9;
    tick.tv_nsec += nsec;
    while (tick.tv_nsec >= SEC_2_NSEC)
    {
        tick.tv_nsec -= SEC_2_NSEC;
        ++tick.tv_sec;
    }
}

void* update_loop(void* threadarg){
    // Extract arguments
    ArgsForThread *arg=(ArgsForThread *)threadarg;
    controller_manager::ControllerManager *manager = arg->manager;
    allegro_hand_ros_control::AllegroHandHWInterface *interface = arg->allegro_hand_hw_interface;

    // Time related stuff
    double loop_rate = 300;
    ros::Duration d(1/loop_rate);
    struct timespec tick;
    clock_gettime(CLOCK_REALTIME, &tick);

    //time for checking overrun
    struct timespec before;
    double overrun_time;

    while(ros::ok()){
        // Get time
        ros::Time this_moment(tick.tv_sec, tick.tv_nsec);
        
        // ROS CONTROL LOOP
        {
            // Read HW
            interface->read(this_moment);
            
            // Controller Manager update
            manager->update(this_moment, d);            

            // Enforce Limits
            // TODO put the enforcer here

            // Write HW
            interface->write(d);
        }

        // Increment Time
        timespecInc(tick, d.nsec);
        
        // check overrun
        clock_gettime(CLOCK_REALTIME, &before);
        overrun_time = (before.tv_sec + double(before.tv_nsec)/1e+9) -  (tick.tv_sec + double(tick.tv_nsec)/1e+9);
        if(overrun_time > 0.0)
        {
            tick.tv_sec=before.tv_sec;
            tick.tv_nsec=before.tv_nsec;
        }

        // Sleep if not overrun
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "allegro_hand_hardware_interface"/*, ros::init_options::AnonymousName*/);

    ros::NodeHandle nh("");

    allegro_hand_ros_control::AllegroHandHWInterface allegro_hand_hw(nh);

    controller_manager::ControllerManager cm(&allegro_hand_hw);
    pthread_t tid;
    ArgsForThread *thread_arg=new ArgsForThread();
    thread_arg->manager = &cm;
    thread_arg->allegro_hand_hw_interface = &allegro_hand_hw;
    pthread_create(&tid, NULL, update_loop, thread_arg);

    ros::Rate r(10);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
}
