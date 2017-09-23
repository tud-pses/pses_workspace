#include <ros/ros.h>
#include <pses_basis/SetMotorLevel.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;

  pses_basis::SetMotorLevel::Request req;
  req.level=6;
  pses_basis::SetMotorLevel::Response res;
  ROS_INFO_STREAM("Sending Request..");
  ros::service::call("set_motor_level", req, res);
  int result = res.was_set;
  ROS_INFO_STREAM("Received Request, Answer: "<<result);


}
