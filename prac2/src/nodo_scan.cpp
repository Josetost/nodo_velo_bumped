#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#define TURNING_TIME 5.0
#define BACKING_TIME 3.0

class ScanGo
{
public:
  BumpGo(): state_(GOING_FORWARD), detected_(0)
  {
    sub_ = n_.subscribe("/scan", 1, &Listener::scanCallback, this);
    pub_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    detected_ = 0;
    if (state_ == GOING_FORWARD):
      //for (int i = 0; i < max_indice_scan)
      //leo el array de scan
      //si una < DIST
        //detected = >0 dependiendo del indice
        //con un funcion imagino
  }

  void step()
  {
    geometry_msgs::Twist cmd;

    switch (state_)
    {
    case GOING_FORWARD:
      cmd.linear.x = 0.25;
      cmd.angular.z = 0;
      if (detected_)
      {
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
      cmd.angular.z = 0.25 * //-1 o +1, depende del giro
      if ((ros::Time::now()-turn_ts_).toSec() > TURNING_TIME )
      {
        state_ = GOING_FORWARD;
        ROS_INFO("TURNING -> GOING_FORWARD");
      }
      break;
    }

    pub_vel_.publish(cmd);
  }

private:
  ros::NodeHandle n_;

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

  ros::Subscriber sub_bumber_;
  ros::Publisher pub_vel_;

  geometry_msgs::Twist cmd;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "bumpgo");

  BumpGo bumpgo;

  ros::Rate loop_rate(20);

  while (ros::ok())
  {
    bumpgo.step();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
