#include <laser_tag.h>
#include <ros/ros.h>
#include <laser_tag/TagActionObs.h>
#include <laser_tag_world.h>

// for tests
#include <iostream>

using namespace despot;

#define TAG 4
#define DEFAULT_NOISE_SIGMA 0.5
#define TERMINATION_OBSERVATION 101
#define NUM_LASER_DIRECTIONS 8

double LaserTagWorld::noise_sigma_ = DEFAULT_NOISE_SIGMA;

bool LaserTagWorld::Connect(){

	// initialize ROS node
	int argc;
	char ** argv;
	ros::init(argc, argv, "test_laser_tag");
	nh = ros::NodeHandlePtr(new ros::NodeHandle);

	// get laser's noise sigma
	if (nh->getParam("/robot/youbot_interface/noise_sigma", noise_sigma_))
	{
	  ROS_INFO("Initialized laser with noise: %f stddev.", noise_sigma_);
	}
	else
	{
	  ROS_INFO("Initialized laser with default noise: %f stddev.", noise_sigma_);
	}

	// wait for laser tag controller service to show up (blocking call)
	ros::service::waitForService("laser_tag_action_obs", -1);

	// setup service client
	client = nh->serviceClient<laser_tag::TagActionObs>("laser_tag_action_obs");
}

//Initialize or reset the environment (for simulators or POMDP world only), return the start state of the system if applicable
State* LaserTagWorld::Initialize(){
	return NULL;
}

//Get the state of the system (only applicable for simulators or POMDP world)
State* LaserTagWorld::GetCurrentState(){
	return NULL;
}

//Send action to be executed by the system, receive observations terminal signals from the system
bool LaserTagWorld::ExecuteAction(ACT_TYPE action, OBS_TYPE& obs){

	/* laser_tag::TagActionObs is a ROS service that takes in an action (integer) 
	 * and outputs observations (8 intergers) after executing the action. If the 
	 * 'Tag' action is called, it returns a boolean 'tag_success' with the outcome.
	 */

	laser_tag::TagActionObs srv;
	srv.request.action = (int) action; // actions: 0 - North, 1 - East, 2 - South, 3 - West, 4 - Tag
	if (client.call(srv))
	{

	  	// successful tag
		if(action == TAG && srv.response.tag_success == true)
		{
			obs=(OBS_TYPE)0;
			for (int dir = 0; dir < NUM_LASER_DIRECTIONS; dir++) {
				LaserTag::SetReading(obs, TERMINATION_OBSERVATION, dir);
			}
			return 1; // exit
		}
		// continue to observe
		else 
		{
			// observations after executing action
			std::vector<int> laser_obs = srv.response.observations;

			// print observations 
			ROS_INFO("Laser Observations");
			ROS_INFO("North: %d"    , laser_obs[0]);
			ROS_INFO("East: %d"     , laser_obs[1]);
			ROS_INFO("South: %d"    , laser_obs[2]);
			ROS_INFO("West: %d"     , laser_obs[3]);
			ROS_INFO("NorthEast: %d", laser_obs[4]);
			ROS_INFO("SouthEast: %d", laser_obs[5]);
			ROS_INFO("SouthWest: %d", laser_obs[6]);
			ROS_INFO("NorthWest: %d", laser_obs[7]);

			for (int dir = 0; dir < 8; dir++) {
			LaserTag::SetReading(obs, laser_obs[dir], dir);
			}
			return 0; // continue
		}
	}
	else
	{
		ROS_ERROR("Failed to execute action & receive observations. Something went wrong with the robot controller!");
		return 0; // continue
	}
}

