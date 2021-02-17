// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "geometry_msgs/Twist.h"
#include "kobuki_msgs/BumperEvent.h"

class Listener
{
public:
  Listener(): n_()
  {
    sub_ = n_.subscribe("message", 1, &Listener::messageCallback, this);
  }

  void messageCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
  {
    ROS_INFO("Message: [%ld]", msg->data);
  }

private:
  ros::NodeHandle n_;
  ros::Subscriber sub_;
};

class Communicator
{
public:
  Listener(): n_()
  {
    pub_ = n_.advertise<geometry_msgs::float>("/mobile_base/commands/velocity", 1);
  }

  void messageCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    ROS_INFO("Message: [%ld]", msg->data);
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "num_subscriber");
  ros::NodeHandle n;

  Listener listener;
  Communicator com;

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    geometry_msgs::Twist vel;
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    vel.linear.x = 0.5;
    if (msg->state == kobuki_msgs::BumperEvent::PRESSED)
    {
      vel.linear.x = 0;
    }
    pub.publish(vel);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
