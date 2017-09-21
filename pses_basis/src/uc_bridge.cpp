#include "ros/ros.h"
#include "pses_basis/SetMotorLevel.h"
#include <pses_basis/communication.h>
#include <pses_basis/communicationconfig.h>
#include <ros/package.h>

bool setMotorLevel(pses_basis::SetMotorLevel::Request& req,
                   pses_basis::SetMotorLevel::Response& res, Communication* com)
{
  std::string cmd = "Drive Forward";
  if (req.level < 0)
    cmd = "Drive Backward";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  input.insertParameter("speed", "uint8_t", req.level);
  res.was_set = com->sendCommand(cmd, input, output);
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uc_bridge");
  ros::NodeHandle nh;
  std::string typesPath = ros::package::getPath("pses_basis") + "/data/";
  Communication com(typesPath);
  // Parameter::ParameterMap input;
  // input.insertParameter("speed", "uint8_t", 5);
  // ROS_INFO_STREAM(input.getParameterValue<int>("speed")<<input.getParameter("speed")->getName()<<input.getParameter("speed")->getType());

  ros::ServiceServer service =
      nh.advertiseService<pses_basis::SetMotorLevel::Request,
                          pses_basis::SetMotorLevel::Response>(
          "set_motor_level", std::bind(setMotorLevel, std::placeholders::_1,
                                       std::placeholders::_2, &com));

  // CommunicationConfig tf(typesPath);
  // tf.readDataTypes();
  // tf.readGeneralSyntax();
  // tf.readCommands();
  // Communication com(typesPath);

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
