#include <ros/ros.h>
#include "simple_rock_sample.h"
#include "simple_rock_sample_world.h"
#include <despot/util/seeds.h>

// for tests
#include <iostream>

using namespace despot;

SimpleRockSampleWorld::SimpleRockSampleWorld(DSPOMDP* model) :
		model_(model) {
	random_ = Random(Seeds::Next());
}

SimpleRockSampleWorld::~SimpleRockSampleWorld() {
}


bool SimpleRockSampleWorld::Connect(){
	return true;
}

//Initialize or reset the environment (for simulators or POMDP world only), return the start state of the system if applicable
State* SimpleRockSampleWorld::Initialize(){
	state_ = model_->CreateStartState();
	return state_;
}

//Get the state of the system (only applicable for simulators or POMDP world)
State* SimpleRockSampleWorld::GetCurrentState(){
	return state_;
}

//Send action to be executed by the system, receive observations terminal signals from the system
bool SimpleRockSampleWorld::ExecuteAction(ACT_TYPE action, OBS_TYPE& obs){
 	bool terminal = model_->Step(*state_, random_.NextDouble(), action,
			step_reward_, obs);
	return terminal;
}

