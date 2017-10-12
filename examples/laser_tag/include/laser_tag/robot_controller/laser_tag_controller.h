#include <iostream>
#include <math.h>
#include <string>
#include <vector>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <laser_tag/YoubotActionObs.h>
#include <laser_tag/TagActionObs.h>

class LaserTagController
{
	public:
	  LaserTagController();
	  enum DIRECTION
	  {
	    NORTH     =  0, 
	    EAST      =  1,
	    SOUTH     =  2,
	    WEST      =  3,
	    STAY      =  4,
	  };
	  
	private:	  
	  ros::NodeHandle nh_;

	  ros::ServiceClient robot_client_;
	  ros::ServiceClient target_client_;
          ros::Subscriber robot_pose_sub_;
          ros::Subscriber target_pose_sub_;

	  ros::Subscriber target_laser_sub_;
	  ros::ServiceServer control_srv_;
	  
	  std::vector<float> target_laser_readings_;
	  std::vector<int> robot_obs_, target_obs_;
	  std::vector<int> robot_position_, target_position_;
	  
	  void laser_cb(const sensor_msgs::LaserScan::ConstPtr& scan);
	  void target_pose_cb(const nav_msgs::Odometry::ConstPtr& odom);
	  void robot_pose_cb(const nav_msgs::Odometry::ConstPtr& odom);
	  
	  bool LaserTagActionObs(laser_tag::TagActionObs::Request &req, laser_tag::TagActionObs::Response &res);

	  DIRECTION TargetNextAction(void);
	  bool CheckCollision(DIRECTION);
	  void RobotSrvCall(int action);
	  void TargetSrvCall(DIRECTION action);
	  
	  std::string ActionToString(int action);
};
