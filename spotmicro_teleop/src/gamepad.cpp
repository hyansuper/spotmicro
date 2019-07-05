#include <ros/ros.h>
#include "spotmicro_teleop/gamepad.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spotmicro_teleop");
  ros::NodeHandle n, param_n("~");
  spotmicro_teleop::Gamepad pad(&n, &param_n);
  double prate, vrate;
  param_n.param("rate", prate, 30.0);
  ros::Rate rate(prate);
  while(ros::ok()) {
  	pad.update();
  	rate.sleep();
  	ros::spinOnce();
  }
  return 0;
}