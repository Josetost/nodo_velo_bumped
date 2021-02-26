#include "ros/ros.h"
#include "ScanGo.hpp"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#define TURNING_TIME 5.0
#define BACKING_TIME 3.0

void ScanGo::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int sz = msg->ranges.size();
    int diff_turn = (int)(M_PI/4)/-msg->angle_increment;
    detected_ = NO_TURN;
    if (state_ == GOING_FORWARD)
    {
      if (msg->ranges[sz/2 - diff_turn] <= DIST_MIN)
      {
        detected_ = LEFT;
      }
      else if (msg->ranges[sz/2] <= DIST_MIN)
      {
        detected_ = CENTER;
      }
      else if (msg->ranges[sz/2 + diff_turn] <= DIST_MIN)
      {
        detected_ = RIGHT;
      }
    }
}

void ScanGo::step()
{
    geometry_msgs::Twist cmd;

    switch (state_)
    {
    case GOING_FORWARD:
      cmd.linear.x = 0.25;
      cmd.angular.z = 0;
      if (detected_)
      {
        ROS_INFO("DETECTED: %d", detected_);
        press_ts_ = ros::Time::now();
        state_ = GOING_BACK;
        ROS_INFO("GOING_FORWARD -> GOING_BACK");
      }
      break;

    case GOING_BACK:
      cmd.linear.x = -0.25;
      cmd.angular.z = 0;

      if ((ros::Time::now() - press_ts_).toSec() > BACKING_TIME )
      {
        turn_ts_ = ros::Time::now();
        state_ = TURNING;
        ROS_INFO("GOING_BACK -> TURNING");
      }
      break;

    case TURNING:
      cmd.linear.x = 0;
      cmd.angular.z = 0.25;  //* -1 o +1, depende del giro
      if ((ros::Time::now()-turn_ts_).toSec() > TURNING_TIME )
      {
        state_ = GOING_FORWARD;
        ROS_INFO("TURNING -> GOING_FORWARD");
      }
      break;
    }

    pub_.publish(cmd);
}
