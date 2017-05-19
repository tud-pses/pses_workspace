#include "ros/ros.h"
#include "pses_basis/SetMotorLevel.h"
#include "pses_basis/serialinterface.h"

bool setMotorLevel(pses_basis::SetMotorLevel::Request& req,
         pses_basis::SetMotorLevel::Response& res)
{
  res.was_set = true;
  ROS_DEBUG("Motor level was set to: %d", req.level);
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uc_bridge");
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("set_motor_level", setMotorLevel);
  //ROS_INFO_STREAM("tut was ...");
  SerialInterface& si = SerialInterface::instance();
  int result = si.connect();
  ROS_INFO_STREAM("ERROR CODE: "<<result);
  //DIR *pDir = opendir("/dev/serial/by-id");
  //if(d == NULL) //Couldn't open directory
  //  return 0;




  ros::spin();

  return 0;
}
