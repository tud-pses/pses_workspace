#include "ros/ros.h"
#include "pses_basis/SetMotorLevel.h"
#include "pses_basis/serialinterface.h"
#include "pses_basis/threaddispatcher.h"

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
  /*
  SerialInterface& si = SerialInterface::instance();
  try {
    si.connect();
  }catch(std::exception& e){
    ROS_ERROR("%s",e.what());
  }
  ThreadDispatcher td;
  td.startReading("\x03");
  std::string msg = std::string("?ID\n");
  si.send(msg);
  ros::Duration(0.5).sleep();
  msg = "!DAQ GRP 1 ~TS=10 ~AVG AX AY AZ\n";
  si.send(msg);
  ros::Duration(0.5).sleep();
  msg = "!DAQ START\n";
  si.send(msg);
  //ros::Duration(5.0).sleep();
  //msg = "!DAQ STOP\n";
  //si.send(msg);
  */

  ros::spin();

  return 0;
}
