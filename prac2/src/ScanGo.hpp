#ifndef SCANGO_H_INCLUDED
#define SCANGO_H_INCLUDED

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

class ScanGo
{
public:
  ScanGo(): state_(GOING_FORWARD), detected_(NO_TURN)
  {
    sub_ = n_.subscribe("/scan", 1, &ScanGo::scanCallback, this);
    pub_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void step();

private:
  ros::NodeHandle n_;

  const float DIST_MIN = 1;

  static const int GOING_FORWARD   = 0;
  static const int GOING_BACK   = 1;
  static const int TURNING     = 2;

  int state_;

  static const int NO_TURN  = 0;
  static const int LEFT   = 1;
  static const int CENTER     = 2;
  static const int RIGHT     = 3;

  int detected_;

  ros::Time press_ts_;
  ros::Time turn_ts_;

  ros::Subscriber sub_;
  ros::Publisher pub_;
};


#endif // SCANGO_H_INCLUDED