#include <ros/ros.h>
#include <pses_basis/SetMotorLevel.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;

  ROS_INFO_STREAM(ros::service::exists("set_motor_level", true));
  pses_basis::SetMotorLevel::Request req;
  req.level=6;
  pses_basis::SetMotorLevel::Response res;
  ros::service::call("set_motor_level", req, res);

  ROS_INFO("Hello world!");
}
