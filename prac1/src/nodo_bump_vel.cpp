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
    sub_ = n_.subscribe("/mobile_base/events/bumper", 1, &Listener::Callback, this);
  }

  void Callback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
  {
    status = msg->state;
  }
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
