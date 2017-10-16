#include "youbot_interface.h"

#define TRANS_GOAL_TOLERANCE 0.05f  // reach target with +-5cm deg tolerance
#define TRANS_SPEED_FACTOR 0.8f

#define CONTROL_RATE 10 // hz
#define NUM_LASER_DIRECTIONS 8
#define CONTROL_TIMEOUT 6 // seconds

#define RANDOM_SEED 45
#define NOISE_SIGMA 0.5

// From despot/src/util/util.cpp
double erf(double x) {
  // constants
  double a1 = 0.254829592;
  double a2 = -0.284496736;
  double a3 = 1.421413741;
  double a4 = -1.453152027;
  double a5 = 1.061405429;
  double p = 0.3275911;
  // Save the sign of x
  int sign = 1;
  if (x < 0)
    sign = -1;
  x = fabs(x);
  // A&S formula 7.1.26
  double t = 1.0 / (1.0 + p * x);
  double y = 1.0
    - (((((a5 * t + a4) * t) + a3) * t + a2) * t + a1) * t * exp(-x * x);

  return sign * y;
}

// CDF of the normal distribution
double gausscdf(double x, double mean, double sigma) {
  return 0.5 * (1 + erf((x - mean) / (sqrt(2) * sigma)));
}


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

  srand(RANDOM_SEED);
}

void YoubotInterface::base_pose_cb(const nav_msgs::Odometry::ConstPtr& odom)
{
  base_pose_ = odom->pose.pose;
}


void YoubotInterface::laser_cb(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // Despot expects laser readings of empty cells (excluding the robot's cell) rounded to the nearest meter.
  // Raw laser readings go from -180deg to 180deg with 0deg = North
  
  std::vector<double> raw_readings;
  raw_readings.resize(8);

  raw_readings[SOUTH]     = scan->ranges[0] + 0.45;
  raw_readings[SOUTHEAST] = scan->ranges[1] + 0.45 * sqrt(2.0);
  raw_readings[EAST]      = scan->ranges[2] + 0.45;
  raw_readings[NORTHEAST] = scan->ranges[3] + 0.45 * sqrt(2.0);
  raw_readings[NORTH]     = scan->ranges[4] + 0.45;
  raw_readings[NORTHWEST] = scan->ranges[5] + 0.45 * sqrt(2.0);
  raw_readings[WEST]      = scan->ranges[6] + 0.45;
  raw_readings[SOUTHWEST] = scan->ranges[7] + 0.45 * sqrt(2.0);

  // sensor gaussian noise model
  double unit_size_ = 1;
  for (int d = 0; d < 8; d++) {
    double r = ((double) rand() / (RAND_MAX));
    double accum_p = 0.0f;
    double dist = raw_readings[d]; 
    int selected=-1;
    int reading = 0;
    for (reading = 0; reading < dist / unit_size_; reading++) {
      double min_noise = reading * unit_size_ - dist;
      double max_noise = std::min(dist, (reading + 1) * unit_size_) - dist;
      double prob =
        2
          * (gausscdf(max_noise, 0, NOISE_SIGMA)
            - (reading > 0 ?
              gausscdf(min_noise, 0, NOISE_SIGMA) : 0 ));

      accum_p += prob;

      if (accum_p > r && selected==-1)
        selected=reading;
    }

    laser_readings_[d] = selected;
  }
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
  cmd.angular.z = 0.0f;
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
  cmd.angular.z = 0.0f;
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
  else if (cmd == "Stay")
  {
    target_x = base_x;
    target_y = base_y;
    ROS_INFO("Stay"); 
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
