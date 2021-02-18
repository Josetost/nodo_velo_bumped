#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "geometry_msgs/Twist.h"
#include "kobuki_msgs/BumperEvent.h"

class Listener
{
public:

  Listener(): n_()
  {
    sub_ = n_.subscribe("/mobile_base/events/bumper", 1, &Listener::getStatus, this);
  }

  // Guardamos el estado en la variable status
  void getStatus(const kobuki_msgs::BumperEvent::ConstPtr& msg)
  {
    status = msg->state;
  }

  // Vemos si esta en contacto el sensor.
  bool pressed()
  {
    return (status == kobuki_msgs::BumperEvent::PRESSED);
  }

private:
  ros::NodeHandle n_;
  ros::Subscriber sub_;
  int status;
};

class Communicator
{
public:

  Communicator(): n_()
  {
    pub_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  }

  // Publica la velocidad
  void publishCom(const geometry_msgs::Twist vel)
  {
    pub_.publish(vel);
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "num_subscriber");
  ros::NodeHandle n;

  // Creamos el comunicador y el listener.
  Listener listener;
  Communicator com;

  geometry_msgs::Twist vel;

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    if (listener.pressed())
    {
      vel.linear.x = 0;
    }
    else
    {
      vel.linear.x = 0.5;
    }
    
    com.publishCom(vel);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
