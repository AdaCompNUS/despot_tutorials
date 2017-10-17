#include <laser_tag.h>
#include <despot/interface/world.h>
#include <ros/ros.h>
using namespace despot;

class LaserTagWorld: public World {
public:
    ros::NodeHandlePtr nh;
    ros::ServiceClient client;

    virtual bool Connect();

    //Initialize or reset the environment (for simulators or POMDP world only), return the start state of the system if applicable
    virtual State* Initialize();

    //Get the state of the system (only applicable for simulators or POMDP world)
    virtual State* GetCurrentState();

    //Send action to be executed by the system, receive observations terminal signals from the system
    virtual bool ExecuteAction(ACT_TYPE action, OBS_TYPE& obs);

    static double noise_sigma_;
};
