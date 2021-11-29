#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>
#include "coppelia_comm/ballbot_states.h"
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_msgs/mpc_flattened_controller.h>
#include <ocs2_mpc/SystemObservation.h>

class coppeliasimCOMM{
public:
    coppeliasimCOMM();
private:
    // Declare sub & pub
    ros::NodeHandle node_handle_;
    ros::Publisher states_pub_;
    ros::Publisher mpc_pub_;
    ros::Subscriber raw_states_sub_;
    ros::Subscriber mpc_policy_sub_;
    
    // Declare functions
    //void stateCallback(const std_msgs::Float64MultiArray::ConstPtr& raw_state_msg);
    void mpcCallback(const ocs2_msgs::mpc_flattened_controller::ConstPtr& mpc_msg);
    
    bool reset_flag = true;
};

coppeliasimCOMM::coppeliasimCOMM():node_handle_("~"){
    // Init ROS related
    ROS_INFO("Inititalizing coppeliasim COMM Node...");

    // Subscribe to realsense topic
    //raw_states_sub_ = node_handle_.subscribe("/coppeliasim/ballbot/raw_states", 1, &coppeliasimCOMM::stateCallback, this); 
    //mpc_policy_sub_ = node_handle_.subscribe("/ballbot_mpc_policy", 1, &coppeliasimCOMM::mpcCallback, this); 
    mpc_policy_sub_ = node_handle_.subscribe("/cartpole_mpc_policy", 1, &coppeliasimCOMM::mpcCallback, this); 
    
    // Publish Init
    //std::string state_topic;
    //node_handle_.param<std::string>("state_topic", state_topic, "/ballbot_mpc_observation");
    //ROS_INFO("state_topic: %s", state_topic.c_str());
    //states_pub_ = node_handle_.advertise<ocs2_msgs::mpc_observation>(state_topic, 1);
    
    std::string mpc_topic;
    //node_handle_.param<std::string>("mpc_topic", mpc_topic, "/coppeliasim/ballbot/mpc_policy");
    node_handle_.param<std::string>("mpc_topic", mpc_topic, "/coppeliasim/cartpole/mpc_policy");
    ROS_INFO("mpc_topic: %s", mpc_topic.c_str());
    mpc_pub_ = node_handle_.advertise<std_msgs::Float64MultiArray>(mpc_topic, 1);
}

/*
void coppeliasimCOMM::stateCallback(const std_msgs::Float64MultiArray::ConstPtr& raw_state_msg){
    //ROS_INFO("callback"); 
    //std::cout << std::setprecision(10) << raw_state_msg->data[0] << std::endl;
    
    ocs2_msgs::mpc_observation state_msg;

    std::vector<double> raw_data = raw_state_msg->data;
    
    state_msg.time = raw_data[1];
    state_msg.mode = 0;

    
    // States:
    // [px,   py,   thetaz,  thetay, thetax, px_dot, py_dot, thetaz_dot, thetay_dot, thetax_dot]
    // [phix, phiy, yaw_abs, pitch,  roll,   dphix,  dphiy,  dyaw_abs,   dpitch,     droll]
    
    state_msg.state.value.push_back((raw_data[17])); //phix
    state_msg.state.value.push_back((raw_data[8])); //phiy
    state_msg.state.value.push_back((raw_data[22])); //yaw_abs
    state_msg.state.value.push_back((raw_data[13])); //pitch
    state_msg.state.value.push_back((raw_data[4])); //roll
    state_msg.state.value.push_back((raw_data[19])); //dphix
    state_msg.state.value.push_back((raw_data[10])); //dphiy
    state_msg.state.value.push_back((raw_data[24])); //dyaw_abs
    state_msg.state.value.push_back((raw_data[15])); //dpitch
    state_msg.state.value.push_back((raw_data[6])); //droll

    // Inputs:
    // [torque wheel1, torque wheel2, torque wheel3]
    // [whl0_tor,      whl1_tor,      whl2_tor]
    
    if (raw_data.size()>43){
    	//num_of_wheels = 4;
        state_msg.input.value.push_back((raw_data[33])); //whl0_tor
        state_msg.input.value.push_back((raw_data[34])); //whl1_tor
        state_msg.input.value.push_back((raw_data[35])); //whl2_tor
        state_msg.input.value.push_back((raw_data[36])); //whl3_tor
    }else{
    	//num_of_wheels = 3;     
        state_msg.input.value.push_back((raw_data[31])); //whl0_tor
        state_msg.input.value.push_back((raw_data[32])); //whl1_tor
        state_msg.input.value.push_back((raw_data[33])); //whl2_tor
     }
        
    states_pub_.publish(state_msg);
}
*/


void coppeliasimCOMM::mpcCallback(const ocs2_msgs::mpc_flattened_controller::ConstPtr& mpc_msg){
    //ROS_INFO("callback"); 
    //std::cout << mpc_msg->stateTrajectory[0] << std::endl;
    
    std_msgs::Float64MultiArray out_msg;
    std::vector<float> stateRef = mpc_msg->stateTrajectory[0].value;
    std::vector<float> inputRef = mpc_msg->inputTrajectory[0].value;
    
    for(size_t i=0;i<stateRef.size();i++){
       out_msg.data.push_back(double(stateRef[i]));
    }
    
    for(size_t i=0;i<inputRef.size();i++){
       out_msg.data.push_back(double(inputRef[i]));
    }
    
    mpc_pub_.publish(out_msg);
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "coppeliasimCOMM");
    
    coppeliasimCOMM node;
    
    ros::spin();
    return 0;
 }
