#include "ros/ros.h"
#include "ScanGo.hpp"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scango");

  ScanGo scango;

  ros::NodeHandle n;

  ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>( "visualization_marker", 0 );

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    scango.step();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
