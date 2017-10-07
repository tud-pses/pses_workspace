#ifndef SERVICEFUNCTIONS_H
#define SERVICEFUNCTIONS_H

#include <ros/ros.h>
#include <pses_basis/communication.h>
// service includes begin
#include <pses_basis/SetMotorLevel.h>
#include <pses_basis/SetSteering.h>
#include <pses_basis/DeleteGroup.h>
#include <pses_basis/GetControllerID.h>
#include <pses_basis/GetDAQStatus.h>
#include <pses_basis/GetFirmwareVersion.h>
#include <pses_basis/GetInfoAllGroups.h>
#include <pses_basis/GetInfoGroup.h>
#include <pses_basis/GetKinectStatus.h>
#include <pses_basis/GetMotorLevel.h>
#include <pses_basis/GetSessionID.h>
#include <pses_basis/GetSteeringLevel.h>
#include <pses_basis/ResetController.h>
#include <pses_basis/SetSessionID.h>
#include <pses_basis/ToggleBrakes.h>
#include <pses_basis/ToggleDAQ.h>
#include <pses_basis/ToggleGroup.h>
#include <pses_basis/ToggleKinect.h>
#include <pses_basis/ToggleMotor.h>
// service includes end

bool setMotorLevel(pses_basis::SetMotorLevel::Request& req,
                   pses_basis::SetMotorLevel::Response& res, Communication* com)
{
  std::string cmd = "Drive Forward";
  short level = req.level;
  if (req.level < 0)
  {
    cmd = "Drive Backward";
    level = -req.level;
  }

  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  input.insertParameter("speed", "int16_t", level);
  res.was_set = com->sendCommand(cmd, input, output);
  return true;
}

bool setSteeringLevel(pses_basis::SetSteering::Request& req,
                   pses_basis::SetSteering::Response& res, Communication* com)
{
  std::string cmd = "Set Steering Level";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  input.insertParameter("steering", "int16_t", req.steering);
  res.was_set = com->sendCommand(cmd, input, output);
  return true;
}

bool deleteGroup(pses_basis::DeleteGroup::Request& req,
                   pses_basis::DeleteGroup::Response& res, Communication* com)
{
  std::string cmd = "Delete Group";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  input.insertParameter("grp_nr", "uint8_t", req.group_number);
  res.was_set = com->sendCommand(cmd, input, output);
  return true;
}

bool getControllerID(pses_basis::GetControllerID::Request& req,
                   pses_basis::GetControllerID::Response& res, Communication* com)
{
  std::string cmd = "Get Controller ID";

  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  res.answer_received = com->sendCommand(cmd, input, output);
  output.getParameterValue("ID",res.ID);
  return true;
}

bool getDAQStatus(pses_basis::GetDAQStatus::Request& req,
                   pses_basis::GetDAQStatus::Response& res, Communication* com)
{
  std::string cmd = "Is DAQ Started";

  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  res.answer_received = com->sendCommand(cmd, input, output);
  output.getParameterValue("info",res.status);
  return true;
}

bool getFirmwareVersion(pses_basis::GetFirmwareVersion::Request& req,
                   pses_basis::GetFirmwareVersion::Response& res, Communication* com)
{
  std::string cmd = "Get Firmware Version";

  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  res.answer_received = com->sendCommand(cmd, input, output);
  output.getParameterValue("version",res.version);
  return true;
}

bool getInfoAllGroups(pses_basis::GetInfoAllGroups::Request& req,
                   pses_basis::GetInfoAllGroups::Response& res, Communication* com)
{
  std::string cmd = "All Groups Info";

  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  res.answer_received = com->sendCommand(cmd, input, output);
  output.getParameterValue("info",res.info);
  return true;
}

bool getInfoGroup(pses_basis::GetInfoGroup::Request& req,
                   pses_basis::GetInfoGroup::Response& res, Communication* com)
{
  std::string cmd = "Group Info";

  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  input.insertParameter("grp_nr", "uint8_t",req.group_number);
  res.answer_received = com->sendCommand(cmd, input, output);
  output.getParameterValue("info",res.info);
  return true;
}

bool getKinectStatus(pses_basis::GetKinectStatus::Request& req,
                   pses_basis::GetKinectStatus::Response& res, Communication* com)
{
  std::string cmd = "Get Kinect Status";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  res.answer_received = com->sendCommand(cmd, input, output);
  output.getParameterValue("kinect_stat",res.status);
  return true;
}

bool getMotorLevel(pses_basis::GetMotorLevel::Request& req,
                   pses_basis::GetMotorLevel::Response& res, Communication* com)
{
  std::string cmd = "Get Motor Level";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  res.answer_received = com->sendCommand(cmd, input, output);
  std::string direction;
  short level;
  output.getParameterValue("direction",direction);
  output.getParameterValue("level",level);
  if(direction.compare("F")!=0){
    level=-level;
  }
  res.level=level;
  return true;
}

bool getSessionID(pses_basis::GetSessionID::Request& req,
                   pses_basis::GetSessionID::Response& res, Communication* com)
{
  std::string cmd = "Get Session ID";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  res.answer_received = com->sendCommand(cmd, input, output);
  output.getParameterValue("ID",res.SID);
  return true;
}

bool getSteeringLevel(pses_basis::GetSteeringLevel::Request& req,
                   pses_basis::GetSteeringLevel::Response& res, Communication* com)
{
  std::string cmd = "Get Steering Level";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  res.answer_received = com->sendCommand(cmd, input, output);
  output.getParameterValue("level",res.level);
  return true;
}

bool resetController(pses_basis::ResetController::Request& req,
                   pses_basis::ResetController::Response& res, Communication* com)
{
  std::string cmd = "Reset Controller";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  res.was_set = com->sendCommand(cmd, input, output);
  return true;
}

bool setSessionID(pses_basis::SetSessionID::Request& req,
                   pses_basis::SetSessionID::Response& res, Communication* com)
{
  std::string cmd = "Set Session ID";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  input.insertParameter("ID", "int32_t", req.sid);
  res.was_set = com->sendCommand(cmd, input, output);
  return true;
}

bool toggleBrakes(pses_basis::ToggleBrakes::Request& req,
                   pses_basis::ToggleBrakes::Response& res, Communication* com)
{
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  std::string cmd;
  if(req.brakes_on){
    cmd = "Full Stop";
  }else{
    cmd = "Drive Forward";
    short level = 0;
    input.insertParameter("speed", "int16_t", level);
  }
  res.was_set = com->sendCommand(cmd, input, output);
  return true;
}

bool toggleDAQ(pses_basis::ToggleDAQ::Request& req,
                   pses_basis::ToggleDAQ::Response& res, Communication* com)
{
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  std::string cmd;
  if(req.DAQ_on){
    cmd = "Start DAQ";
  }else{
    cmd = "Stop DAQ";
  }
  res.was_set = com->sendCommand(cmd, input, output);
  return true;
}

bool toggleGroup(pses_basis::ToggleGroup::Request& req,
                   pses_basis::ToggleGroup::Response& res, Communication* com)
{
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  std::string cmd;
  if(req.group_on){
    cmd = "Activate Group";
  }else{
    cmd = "Deactivate Group";
  }
  input.insertParameter("grp_nr", "uint8_t", req.group_number);
  res.was_set = com->sendCommand(cmd, input, output);
  return true;
}

bool toggleKinect(pses_basis::ToggleKinect::Request& req,
                   pses_basis::ToggleKinect::Response& res, Communication* com)
{
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  std::string cmd;
  if(req.kinect_on){
    cmd = "Toggle Kinect On";
  }else{
    cmd = "Toggle Kinect Off";
  }
  res.was_set = com->sendCommand(cmd, input, output);
  return true;
}

bool toggleMotor(pses_basis::ToggleMotor::Request& req,
                   pses_basis::ToggleMotor::Response& res, Communication* com)
{
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  std::string cmd;
  if(req.motor_on){
    cmd = "Drive Forward";
    short level = 0;
    input.insertParameter("speed", "int16_t", level);
  }else{
    cmd = "Motor Off";
  }
  res.was_set = com->sendCommand(cmd, input, output);
  return true;
}
#endif // SERVICEFUNCTIONS_H
