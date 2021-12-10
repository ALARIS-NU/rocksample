#include "simple_rock_sample.h"
#include <despot/core/globals.h>
#include <despot/interface/pomdp.h>
#include <despot/interface/world.h>
#include <despot/util/random.h>
#include <ros/ros.h>
using namespace despot;

class SimpleRockSampleWorld: public World {
protected:
        //POMDP model shared with despot solver
    DSPOMDP* model_;
    //Random number generator.
    Random random_;

public:
    double step_reward_;

public:
    SimpleRockSampleWorld(DSPOMDP* model);
    virtual ~SimpleRockSampleWorld();
    virtual inline void world_seed(unsigned seed) {
        random_ = Random(seed);
    }    

    bool Connect();

    //Initialize or reset the environment (for simulators or POMDP world only), return the start state of the system if applicable
    State* Initialize();

    //Get the state of the system (only applicable for simulators or POMDP world)
    State* GetCurrentState();

    //Send action to be executed by the system, receive observations terminal signals from the system
    bool ExecuteAction(ACT_TYPE action, OBS_TYPE& obs);
};
