#include "ros/ros.h"
#include "ScanGo.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scango");

  ScanGo scango;

  ros::Rate loop_rate(20);

  while (ros::ok())
  {
    scango.step();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
