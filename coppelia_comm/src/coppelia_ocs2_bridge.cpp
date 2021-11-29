#include <ros/ros.h>
#include "coppelia_comm/ballbot_states.h"
#include <std_msgs/Float64MultiArray.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_msgs/mpc_flattened_controller.h>
#include <ocs2_msgs/mpc_target_trajectories.h>
#include <ocs2_core/Types.h>

using namespace ocs2;

class coppeliasimCOMM{
public:
	coppeliasimCOMM();
private:
	// Declare sub & pub
	ros::NodeHandle node_handle_;
	ros::Subscriber states_sub_;
	ros::Subscriber raw_target_sub_;
	ros::Publisher target_pub_;

	// Declare functions
	void stateCallback(const ocs2_msgs::mpc_observation::ConstPtr& state_msg);
	void targetCallback(const std_msgs::Float64MultiArray::ConstPtr& target_msg);

	vector_t state_temp;
	scalar_t state_time;
	bool state_flag;
	int state_dim;
	int intput_dim;
};

coppeliasimCOMM::coppeliasimCOMM():node_handle_("~"){
	// Init ROS related
	ROS_INFO("Inititalizing coppeliasim COMM Node...");

	//states_sub_ = node_handle_.subscribe("/ballbot_mpc_observation", 1, &coppeliasimCOMM::stateCallback, this); 
	//states_sub_ = node_handle_.subscribe("/cartpole_mpc_observation", 1, &coppeliasimCOMM::stateCallback, this); 
	states_sub_ = node_handle_.subscribe("/pure_planar_mpc_observation", 1, &coppeliasimCOMM::stateCallback, this); 
	raw_target_sub_ = node_handle_.subscribe("/coppeliasim/pure_planar/target_states", 1, &coppeliasimCOMM::targetCallback, this); 
	//target_pub_ = node_handle_.advertise<ocs2_msgs::mpc_target_trajectories>("/ballbot_mpc_target", 1);
	//target_pub_ = node_handle_.advertise<ocs2_msgs::mpc_target_trajectories>("/cartpole_mpc_target", 1);
	target_pub_ = node_handle_.advertise<ocs2_msgs::mpc_target_trajectories>("/pure_planar_mpc_target", 1);

	state_flag = false;
	state_dim = 4; //10
	intput_dim = 1; //3
}

void coppeliasimCOMM::stateCallback(const ocs2_msgs::mpc_observation::ConstPtr& state_msg){
	state_time = state_msg->time;
	//state_temp = state_msg->state.value;
	
	const auto& state = state_msg->state.value;
	state_temp = Eigen::Map<const Eigen::VectorXf>(state.data(), state.size()).cast<scalar_t>();
	
	state_flag = true;
}


void coppeliasimCOMM::targetCallback(const std_msgs::Float64MultiArray::ConstPtr& target_msg){
	//ROS_INFO("callback"); 
	if (state_flag) {
		ocs2_msgs::mpc_target_trajectories out_msg;
		std::vector<double> raw_ref = target_msg->data;
	  
		  const vector_t targetState = [&]() {
		    vector_t targetState = vector_t::Zero(state_dim);
		    targetState(0) = state_temp[0];//raw_ref[0];
		    targetState(1) = raw_ref[1];
		    //targetState(2) = raw_ref[2];
		    //targetState(3) = state_temp[3];
		    //targetState(4) = state_temp[4];
		    return targetState;
		  }();

		// Target reaching duration
		constexpr scalar_t averageSpeed = 2.0/0.127;
		const vector_t deltaPose = (targetState - state_temp).head<2>();//5
		const scalar_t targetTime = state_time + deltaPose.norm() / averageSpeed;

		// Desired time trajectory
		const scalar_array_t timeTrajectory{state_time, targetTime};

		// Desired state trajectory
		const vector_array_t stateTrajectory{state_temp, targetState};

		// Desired input trajectory
		const vector_array_t inputTrajectory(2, vector_t::Zero(intput_dim));

		// time and state
		size_t N = stateTrajectory.size();
		out_msg.timeTrajectory.resize(N);
		out_msg.stateTrajectory.resize(N);

		for (size_t i = 0; i < N; i++) {
			out_msg.timeTrajectory[i] = timeTrajectory[i];

			out_msg.stateTrajectory[i].value =
			std::vector<float>(stateTrajectory[i].data(), stateTrajectory[i].data() + stateTrajectory[i].size());
		}  // end of i loop

		// input
		N = inputTrajectory.size();
		out_msg.inputTrajectory.resize(N);
		for (size_t i = 0; i < N; i++) {
			out_msg.inputTrajectory[i].value =
			std::vector<float>(inputTrajectory[i].data(), inputTrajectory[i].data() + inputTrajectory[i].size());
		}  // end of i loop

		target_pub_.publish(out_msg);
	}
}

int main (int argc, char** argv) {
	ros::init(argc, argv, "coppeliasimCOMM");

	coppeliasimCOMM node;

	ros::spin();
	return 0;
 }
