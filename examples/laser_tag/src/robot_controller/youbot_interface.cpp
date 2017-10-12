#include "youbot_interface.h"

#define TRANS_GOAL_TOLERANCE 0.05f  // reach target with +-5cm deg tolerance
#define TRANS_SPEED_FACTOR 0.8f

#define CONTROL_RATE 10 // hz
#define NUM_LASER_DIRECTIONS 8
#define CONTROL_TIMEOUT 6 // seconds

YoubotInterface::YoubotInterface()
{
  /*
   *  Initialize controller and laser interface for the Youbot robot (without KUKA arm)
   */
  
  laser_readings_.resize(NUM_LASER_DIRECTIONS);
  
  control_srv_ = nh_.advertiseService("youbot_discrete_controller", &YoubotInterface::DiscreteController, this);
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  base_pose_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 30, &YoubotInterface::base_pose_cb, this);
  laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("laser_scan", 30, &YoubotInterface::laser_cb, this);
}

void YoubotInterface::base_pose_cb(const nav_msgs::Odometry::ConstPtr& odom)
{
  base_pose_ = odom->pose.pose;
}

void YoubotInterface::laser_cb(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // Despot expects laser readings of empty cells (excluding the robot's cell) rounded to the nearest meter.
  // Raw laser readings go from -180deg to 180deg with 0deg = North
  
  laser_readings_[SOUTH]     = (int) round(scan->ranges[0] - 0.5f);
  laser_readings_[SOUTHEAST] = (int) round(scan->ranges[1]);
  laser_readings_[EAST]      = (int) round(scan->ranges[2] - 0.5f);
  laser_readings_[NORTHEAST] = (int) round(scan->ranges[3]);
  laser_readings_[NORTH]     = (int) round(scan->ranges[4] - 0.5f);
  laser_readings_[NORTHWEST] = (int) round(scan->ranges[5]);
  laser_readings_[WEST]      = (int) round(scan->ranges[6] - 0.5f);
  laser_readings_[SOUTHWEST] = (int) round(scan->ranges[7]);
}

void YoubotInterface::Goto(float x, float y)
{
  /*
   *  Goto a given pose. No uncertainity involved in executing actions
   */

  float curr_x = base_pose_.position.x;
  float curr_y = base_pose_.position.y;

  float delta_x = x - curr_x;
  float delta_y = y - curr_y;
  geometry_msgs::Twist cmd = geometry_msgs::Twist();
  ros::Rate loop_rate(CONTROL_RATE);

  // stop moving
  cmd.linear.x = 0.0f;
  cmd.linear.y = 0.0f;
  vel_pub_.publish(cmd);

  ros::Time start_time = ros::Time::now();
  while (ros::ok && (fabs(delta_x) > TRANS_GOAL_TOLERANCE || fabs(delta_y) > TRANS_GOAL_TOLERANCE) )
  {
    curr_x = base_pose_.position.x;
    curr_y = base_pose_.position.y;

    delta_x = x - curr_x;
    delta_y = y - curr_y;

    // transform vel to robot coordinate frame (90deg anti-clockwise rotation)
    cmd.linear.x =  delta_y * TRANS_SPEED_FACTOR;
    cmd.linear.y = -delta_x * TRANS_SPEED_FACTOR;
    vel_pub_.publish(cmd);

    ros::Time curr_time = ros::Time::now();
    ros::Duration diff = curr_time - start_time;
    if (diff.toSec() > CONTROL_TIMEOUT)
      break;
    
    loop_rate.sleep();
    ros::spinOnce();
  }
  
  // stop moving
  cmd.linear.x = 0.0f;
  cmd.linear.y = 0.0f;
  vel_pub_.publish(cmd);
  
}

bool YoubotInterface::DiscreteController(laser_tag::YoubotActionObs::Request &req,
					  laser_tag::YoubotActionObs::Response &res)
{
  std::string cmd = req.direction.c_str();

  // always start at the center of a square
  float base_x = floor(base_pose_.position.x) + 0.5f;
  float base_y = floor(base_pose_.position.y) + 0.5f;

  float target_x = base_x;
  float target_y = base_y;
  
  ROS_INFO("Received Command: %s", cmd.c_str());
  
  if (cmd == "North")
  {
    target_x = base_x;
    target_y = base_y + 1.0f;
    ROS_INFO("Going North...");
  }
  else if (cmd == "East")
  {
    target_x = base_x + 1.0f;
    target_y = base_y;
    ROS_INFO("Going East...");
  }
  else if (cmd == "South")
  {
    target_x = base_x;
    target_y = base_y - 1.0f;
    ROS_INFO("Going South...");
  }
  else if (cmd == "West")
  {
    target_x = base_x - 1.0f;
    target_y = base_y;
    ROS_INFO("Going West...");
  }
  else if (cmd == "NE")
  {
    target_x = base_x + 1.0f;
    target_y = base_y + 1.0f;
    ROS_INFO("Going NorthEast...");
  }
  else if (cmd == "SE")
  {
    target_x = base_x + 1.0f;
    target_y = base_y - 1.0f;
    ROS_INFO("Going SouthEast...");
  }
  else if (cmd == "SW")
  {
    target_x = base_x - 1.0f;
    target_y = base_y - 1.0f;
    ROS_INFO("Going SouthWest...");
  }
  else if (cmd == "NW")
  {
    target_x = base_x - 1.0f;
    target_y = base_y + 1.0f;
    ROS_INFO("Going NorthWest"); 
  }
  else
  {
    ROS_ERROR("Invalid direction. Possible directions: North, East, South, West, NE, SE, SW, NW");
    res.laser_readings = laser_readings_;
    return false;
  }
  
  this->Goto(target_x, target_y);
  
  res.laser_readings = laser_readings_;	
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "youbot_interface");
  YoubotInterface youbot_interface;

  // Main Loop
  ros::Rate loop_rate(CONTROL_RATE);
  while (true)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
