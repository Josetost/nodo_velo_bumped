#include "ros/ros.h"
#include "ScanGo.hpp"


#define TURNING_TIME 3.0
#define BACKING_TIME 2.0


void ScanGo::manageMarker(visualization_msgs::MarkerArray a, int color)
{
  //x = d * cost
  //y = d * sint

  for (int i = 0; i < 3; i++)
  {
    vect[i].header.frame_id = "base_link";
    vect[i].header.stamp = ros::Time();
    vect[i].ns = "my_namespace";
    vect[i].id = i+1;
    vect[i].type = visualization_msgs::Marker::SPHERE;
    vect[i].action = visualization_msgs::Marker::ADD;
    vect[i].pose.position.x = DIST_MIN*cos(M_PI/5 - M_PI/5*i);
    vect[i].pose.position.y = DIST_MIN*sin(M_PI/5 - M_PI/5*i);
    vect[i].pose.position.z = 0.0;
    vect[i].pose.orientation.x = 0.0;
    vect[i].pose.orientation.y = 0.0;
    vect[i].pose.orientation.z = 0.0;
    vect[i].pose.orientation.w = 1.0;
    vect[i].scale.x = 0.25;
    vect[i].scale.y = 0.25;
    vect[i].scale.z = 0.25;
    vect[i].color.a = 1.0;
    vect[i].color.b = 0.0;
    vect[i].color.r = 0.0;
    vect[i].color.g = 0.0;

    if (color != 0 && color - 1 == i)
    {
      vect[i].color.r = 1.0;
    }
    else{
      vect[i].color.g = 1.0;
    }
    a.markers[i] = vect[i];

    pub_marker_.publish(a);
  }
}

double ScanGo::randAng(int orientation)
{
  double rand_seed = rand() / (double) RAND_MAX;
  rand_seed = M_PI / 4 + rand_seed * ( M_PI/2 - M_PI / 4 );
  rand_seed = rand_seed / TURNING_TIME;
  if (orientation == LEFT)
  {
    rand_seed *= -1.0;
  }
  return rand_seed;
}

void ScanGo::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int sz = msg->ranges.size();
    int diff_turn = (int)((M_PI/5)/-msg->angle_increment);
    
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
      srand(time(NULL));
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
      cmd.linear.x = -0.1;
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
      cmd.angular.z = randAng(detected_);
      if ((ros::Time::now()-turn_ts_).toSec() > TURNING_TIME )
      {
        state_ = GOING_FORWARD;
        ROS_INFO("TURNING -> GOING_FORWARD");
        detected_ = NO_TURN;
      }
      break;
    }
    manageMarker(array_markers_, detected_);
    pub_.publish(cmd);
}
