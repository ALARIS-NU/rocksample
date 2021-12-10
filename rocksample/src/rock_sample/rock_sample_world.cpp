#include <ros/ros.h>
#include "rock_sample.h"
#include "rock_sample_world.h"

using namespace std;
using namespace despot;

RockSampleWorld::RockSampleWorld(DSPOMDP* model, unsigned seed) :
		model_(model) {
	random_ = Random(seed);
}

RockSampleWorld::~RockSampleWorld() {
}


bool RockSampleWorld::Connect(){
	// initialize ROS node
	int argc;
	char ** argv;
  std::string name = "test_rock_sample";
	ros::init(argc, argv, name);
	nh = ros::NodeHandlePtr(new ros::NodeHandle);
  robot_client_ = nh->serviceClient<rocksample::YoubotActionObs>("/robot/youbot_discrete_controller");

	return true;
}

//Initialize or reset the environment (for simulators or POMDP world only), return the start state of the system if applicable
State* RockSampleWorld::Initialize(){
	state_ = model_->CreateStartState();
	return state_;
}

//Get the state of the system (only applicable for simulators or POMDP world)
State* RockSampleWorld::GetCurrentState(){
	return state_;
}

//Send action to be executed by the system, receive observations terminal signals from the system
bool RockSampleWorld::ExecuteAction(ACT_TYPE action, OBS_TYPE& obs){
	bool terminal = model_->Step(*state_, random_.NextDouble(), action,
			step_reward_, obs);

  switch(obs) {
    case 2:
      ROS_INFO("Observation: None");
      break;
    case 1:
      ROS_INFO("Observation: Good");
      break;
    case 0:
      ROS_INFO("Observation: Bad");
      break;
  }

  if(action < 4) {
    rocksample::YoubotActionObs srv;
    srv.request.direction = this->ActionToString(action);

    if (robot_client_.call(srv)) {
      // /ROS_INFO("Finished executing robot action");
    } else {
      ROS_ERROR("Something went wrong with the robot interface");
    }
  } 

  return terminal;
}

std::string RockSampleWorld::ActionToString(int action)
{
  if (action == 0)
    return "North";
  else if (action == 1)
    return "East";
  else if (action == 2)
    return "South";
  else if (action == 3)
    return "West";
	else
    return "";
}




