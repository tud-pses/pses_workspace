#include "ros/ros.h"
#include "pses_basis/SetMotorLevel.h"
#include <pses_basis/communication.h>
#include <pses_basis/threadfactory.h>
#include <ros/package.h>

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
  ros::ServiceServer service =
      nh.advertiseService("set_motor_level", setMotorLevel);
  std::string typesPath = ros::package::getPath("pses_basis")+ "/data/";
  ThreadFactory tf(typesPath);
  tf.readDataTypes();

  /*
  Communication com;
  try
  {
    com.connect();
    com.startCommunication();
    com.stopCommunication();
    com.disconnect();
  }
  catch (std::exception& e)
  {
    ROS_ERROR("%s", e.what());
  }
  */

  ros::spin();

  return 0;
}
