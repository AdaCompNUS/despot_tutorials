#include <iostream>
#include <math.h>
#include <string>
#include <vector>

//#include <boost/thread/thread.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <laser_tag/YoubotActionObs.h>

class YoubotInterface
{
	public:
	  enum DIRECTION
	  {
	    NORTH     =  0, 
	    EAST      =  1,
	    SOUTH     =  2,
	    WEST      =  3,
	    NORTHEAST =  4,
	    SOUTHEAST =  5,
	    SOUTHWEST =  6,
	    NORTHWEST =  7
	  };
	  
	  YoubotInterface();
	  void Goto(float x, float y);
	 
	  
	private:
	  
	  ros::NodeHandlePtr nh_;

	  ros::Subscriber base_pose_sub_;
      ros::Subscriber laser_sub_;

	  geometry_msgs::Pose base_pose_;
      std::vector<int> laser_readings_;
	  
	  ros::ServiceServer control_srv_;
	  ros::Publisher vel_pub_;

	  float noise_sigma_;

	  void base_pose_cb(const nav_msgs::Odometry::ConstPtr& odom);
	  void laser_cb(const sensor_msgs::LaserScan::ConstPtr& scan);
	  geometry_msgs::Vector3 quat2euler(const geometry_msgs::Quaternion quat);

	  bool DiscreteController(laser_tag::YoubotActionObs::Request &req,
				  laser_tag::YoubotActionObs::Response &res);

};
