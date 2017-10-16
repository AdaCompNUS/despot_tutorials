#include "laser_tag_controller.h"

#define CONTROL_RATE 30
#define RANDOM_SEED 42

LaserTagController::LaserTagController()
{
  target_laser_readings_.resize(4); // North, East, South, West
  robot_position_.resize(2); // x, y
  target_position_.resize(2);
  
  robot_client_ = nh_.serviceClient<laser_tag::YoubotActionObs>("/robot/youbot_discrete_controller");
  target_client_ = nh_.serviceClient<laser_tag::YoubotActionObs>("/target/youbot_discrete_controller");

  target_laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/target/laser_scan", 30, &LaserTagController::laser_cb, this);
  robot_pose_sub_ = nh_.subscribe<nav_msgs::Odometry>("/robot/odom", 30, &LaserTagController::robot_pose_cb, this);
  target_pose_sub_ = nh_.subscribe<nav_msgs::Odometry>("/target/odom", 30, &LaserTagController::target_pose_cb, this);

  control_srv_ = nh_.advertiseService("laser_tag_action_obs", &LaserTagController::LaserTagActionObs, this);

  srand(RANDOM_SEED);
}

void LaserTagController::laser_cb(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // raw target sensor readings for collision avoidance during evasion
  target_laser_readings_[SOUTH] = scan->ranges[0];
  target_laser_readings_[EAST]  = scan->ranges[2];
  target_laser_readings_[NORTH] = scan->ranges[4];
  target_laser_readings_[WEST]  = scan->ranges[6];
}

void LaserTagController::robot_pose_cb(const nav_msgs::Odometry::ConstPtr& odom)
{
  robot_position_[0] = (int) floor(odom->pose.pose.position.x);
  robot_position_[1] = (int) floor(odom->pose.pose.position.y);
}

void LaserTagController::target_pose_cb(const nav_msgs::Odometry::ConstPtr& odom)
{
  target_position_[0] = (int) floor(odom->pose.pose.position.x);
  target_position_[1] = (int) floor(odom->pose.pose.position.y);
}

std::string LaserTagController::ActionToString(int action)
{
  if (action == NORTH)
    return "North";
  else if (action == EAST)
    return "East";
  else if (action == SOUTH)
    return "South";
  else if (action == WEST)
    return "West";
  else if (action == STAY)
    return "Stay";
}

bool LaserTagController::CheckCollision(LaserTagController::DIRECTION direction)
{
  return (target_laser_readings_[direction] < 1.0f);
}

LaserTagController::DIRECTION LaserTagController::TargetNextAction(void)
{
  /*
   * Evasive maneuver 
   */

  
  // generate random float between 0 & 1
  double r = ((double) rand() / (RAND_MAX));

  int delta_x = target_position_[0] - robot_position_[0];
  int delta_y = target_position_[1] - robot_position_[1];

  if (r >= 0.0f && r <= 0.4f)
  {
    if (delta_x == 0) // target is directly below robot
    {
      if (r >= 0.0f && r <= 0.2f)
	return EAST;
      else
	return WEST;
    }
    else if (delta_x < 0) // target is to the right of the robot
    {
      return WEST;
    }
    else if (delta_x > 0) // target is to the left of the robot
    {
      return EAST;
    }
  }

  else if (r > 0.4f && r <= 0.8f)
  {
    if (delta_y == 0) // target is directly next to the robot
    {
      if (r > 0.4f && r <= 0.6f)
	return NORTH;
      else
	return SOUTH;
    }
    else if (delta_y < 0) // target is above the robot
    {
      return SOUTH;
    }
    else if (delta_y > 0) // target is below the robot
    {
      return NORTH;
    }
  }
    
  return STAY;
}


void LaserTagController::RobotSrvCall(int action)
{
  laser_tag::YoubotActionObs srv;
  srv.request.direction = this->ActionToString(action);

  if (robot_client_.call(srv))
    ROS_INFO("Finished executing robot action");
  else
    ROS_ERROR("Something went wrong with the robot interface");

  robot_obs_ = srv.response.laser_readings;
}


void LaserTagController::TargetSrvCall(LaserTagController::DIRECTION action)
{
  laser_tag::YoubotActionObs srv;
  srv.request.direction = this->ActionToString(action);

  if (target_client_.call(srv))
    ROS_INFO("Finished executing target action");
  else
    ROS_ERROR("Something went wrong with the target interface");

  target_obs_ = srv.response.laser_readings;
}


bool LaserTagController::LaserTagActionObs(laser_tag::TagActionObs::Request &req,
					   laser_tag::TagActionObs::Response &res)
{
  // default tag status
  res.tag_success = false;

  // check if action is valid
  if (req.action < 0 || req.action > 4)
  {
    ROS_ERROR("Invalid Action Integer. Valid actions (0-4): 0 - North, 1 - East, 2 - South, 3 - West, 4 - Tag");
    return false;
  }

  // plan evasive action for target
  LaserTagController::DIRECTION next_target_action = this->TargetNextAction();
  next_target_action = (this->CheckCollision(next_target_action) || req.action == 4) ? STAY : next_target_action;
  
  boost::thread thread_target, thread_robot;
  
  // send commands to robot and target 
  if (next_target_action != STAY)
    thread_target = boost::thread(boost::bind(&LaserTagController::TargetSrvCall, this, next_target_action));
  thread_robot = boost::thread(boost::bind(&LaserTagController::RobotSrvCall, this, req.action));

  // wait for robot and target to finish
  if (next_target_action != STAY)
    thread_target.join();
  thread_robot.join();

  // latest observations
  res.observations = robot_obs_;

  // check if Tag
  if (req.action == 4)
  {
    int delta_x = target_position_[0] - robot_position_[0];
    int delta_y = target_position_[1] - robot_position_[1];

    // target has to be in one of the four directions
    if ( (abs(delta_x) + abs(delta_y)) == 1)
    {
      ROS_INFO("!!!!!!!!!!!!!!!TAG!!!!!!!!!!!!!!!!!!!!");
      res.tag_success = true;
    }
    else
    {
      ROS_WARN("Invalid Tag!");
    }
  }

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_tag_controller");
  LaserTagController youbot_interface;
  
  // Main Loop
  ros::Rate loop_rate(CONTROL_RATE);
  while (true)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
