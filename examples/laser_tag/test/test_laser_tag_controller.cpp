#include <ros/ros.h>
#include <laser_tag/TagActionObs.h>

// for tests
#include <iostream>

int main(int argc, char **argv)
{
  // initilize ROS node. Should be done just once
  ros::init(argc, argv, "test_laser_tag");
  ros::NodeHandle nh;
  
  // wait for controller service to show up (waits forever)
  ros::service::waitForService("laser_tag_action_obs", -1);
  
  // setup service client
  ros::ServiceClient client = nh.serviceClient<laser_tag::TagActionObs>("laser_tag_action_obs");

  // send action & receive observation
  laser_tag::TagActionObs srv;
  srv.request.action = 0; // actions: 0 - North, 1 - East, 2 - South, 3 - West, 4 - Tag
  if (client.call(srv))
  {
    // observations after executing actions
    std::vector<int> laser_obs = srv.response.observations;

    // print observations (metric readings rounded to the nearest integer)
    ROS_INFO("Laser Observations");
    ROS_INFO("North: %d"    , laser_obs[0]);
    ROS_INFO("East: %d"     , laser_obs[1]);
    ROS_INFO("South: %d"    , laser_obs[2]);
    ROS_INFO("West: %d"     , laser_obs[3]);
    ROS_INFO("NorthEast: %d", laser_obs[4]);
    ROS_INFO("SouthEast: %d", laser_obs[5]);
    ROS_INFO("SouthWest: %d", laser_obs[6]);
    ROS_INFO("NorthWest: %d", laser_obs[7]);
  }
  else
  {
    ROS_ERROR("Invalid Action OR Invalid Tag");
    return 1;
  }

  return 0;
}
