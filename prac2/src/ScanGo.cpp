#include "ros/ros.h"
#include "ScanGo.hpp"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <random>
#include "visualization_msgs/MarkerArray.h"

#define TURNING_TIME 5.0
#define BACKING_TIME 3.0


void static manageMarker(visualization_msgs::MarkerArray a, int color)
{
  //x = d * cost
  //y = d * sint
  for (int i = 0; i < 3; i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 1;
    marker.pose.position.y = 1;
    marker.pose.position.z = 1;
    marker.pose.orientation.x = (double)1*cos(M_PI/5 - M_PI/5*i);
    marker.pose.orientation.y = (double)1*sin(M_PI/5 - M_PI/5*i);
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.a = 1.0;
    marker.color.b = 0.0;
    if (color != 0 && color - 1 == i)
    {
      marker.color.r = 1.0;
      marker.color.g = 0.0;
    }
    else{
      marker.color.r = 0.0;
      marker.color.g = 1.0;
    }
    a.markers[i] = marker;
  }
}

void ScanGo::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int sz = msg->ranges.size();
    int diff_turn = (int)(M_PI/5)/-msg->angle_increment;
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
    srand(time(NULL));

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
      // rand % (max_number +1 - minimun_number ) + mininum_number
      cmd.angular.z = ((double)rand() / RAND_MAX) * (M_PI/2 - M_PI / 4) + M_PI / 4;
      if (detected_ == LEFT)
      {
        cmd.angular.z *= -1;
      }
      if ((ros::Time::now()-turn_ts_).toSec() > TURNING_TIME )
      {
        state_ = GOING_FORWARD;
        ROS_INFO("TURNING -> GOING_FORWARD");
        detected_ = NO_TURN;

      }
      break;
    }
    manageMarker(array_markers_, detected_);
    pub_marker_.publish(array_markers_);
    pub_.publish(cmd);
}
