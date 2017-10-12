#ifndef SERVICEFUNCTIONS_H
#define SERVICEFUNCTIONS_H

#include <ros/ros.h>
#include <pses_basis/communication.h>
// service includes begin
#include <pses_basis/DeleteGroup.h>
#include <pses_basis/GetControllerID.h>
#include <pses_basis/GetDAQStatus.h>
#include <pses_basis/GetFirmwareVersion.h>
#include <pses_basis/GetIMUInfo.h>
#include <pses_basis/GetInfoAllGroups.h>
#include <pses_basis/GetInfoGroup.h>
#include <pses_basis/GetKinectStatus.h>
#include <pses_basis/GetMagASA.h>
#include <pses_basis/GetMagInfo.h>
#include <pses_basis/GetMotorLevel.h>
#include <pses_basis/GetSessionID.h>
#include <pses_basis/GetSteeringLevel.h>
#include <pses_basis/GetUSInfo.h>
#include <pses_basis/ResetController.h>
#include <pses_basis/ResetDMS.h>
#include <pses_basis/SetIMU.h>
#include <pses_basis/SetMag.h>
#include <pses_basis/SetMotorLevel.h>
#include <pses_basis/SetSteering.h>
#include <pses_basis/SetSessionID.h>
#include <pses_basis/SetUS.h>
#include <pses_basis/ToggleBrakes.h>
#include <pses_basis/ToggleDAQ.h>
#include <pses_basis/ToggleDMS.h>
#include <pses_basis/ToggleGroup.h>
#include <pses_basis/ToggleKinect.h>
#include <pses_basis/ToggleMotor.h>
#include <pses_basis/ToggleUS.h>

// service includes end

namespace ServiceFunctions{

bool deleteGroup(pses_basis::DeleteGroup::Request& req,
                   pses_basis::DeleteGroup::Response& res, Communication* com)
{
  res.was_set = false;
  std::string cmd = "Delete Group";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  input.insertParameter("grp_nr", "uint8_t", req.group_number);
  try{
    res.was_set = com->sendCommand(cmd, input, output);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Service 'Delete Group' occured!\n Description: "<<e.what());
    res.was_set = false;
    return false;
  }
  return true;
}

bool getControllerID(pses_basis::GetControllerID::Request& req,
                   pses_basis::GetControllerID::Response& res, Communication* com)
{
  res.answer_received = false;
  std::string cmd = "Get Controller ID";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try{
    res.answer_received = com->sendCommand(cmd, input, output);
    if(!res.answer_received) return false;
    output.getParameterValue("ID",res.ID);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Service 'Get Controller ID' occured!\n Description: "<<e.what());
    res.answer_received = false;
    return false;
  }
  return true;
}

bool getDAQStatus(pses_basis::GetDAQStatus::Request& req,
                   pses_basis::GetDAQStatus::Response& res, Communication* com)
{
  res.answer_received = false;
  std::string cmd = "Is DAQ Started";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try{
    res.answer_received = com->sendCommand(cmd, input, output);
    if(!res.answer_received) return false;
    output.getParameterValue("info",res.status);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Service 'Is DAQ Started' occured!\n Description: "<<e.what());
    res.answer_received = false;
    return false;
  }
  return true;
}

bool getFirmwareVersion(pses_basis::GetFirmwareVersion::Request& req,
                   pses_basis::GetFirmwareVersion::Response& res, Communication* com)
{
  res.answer_received = false;
  std::string cmd = "Get Firmware Version";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try{
    res.answer_received = com->sendCommand(cmd, input, output);
    if(!res.answer_received) return false;
    output.getParameterValue("version",res.version);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Service 'Get Firmware Version' occured!\n Description: "<<e.what());
    res.answer_received = false;
    return false;
  }
  return true;
}

bool getIMUInfo(pses_basis::GetIMUInfo::Request& req,
                   pses_basis::GetIMUInfo::Response& res, Communication* com)
{
  res.answer_received = false;
  std::string cmd = "Get IMU Info";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try{
    res.answer_received = com->sendCommand(cmd, input, output);
    if(!res.answer_received) return false;
    output.getParameterValue("info",res.info);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Service 'Get IMU info' occured!\n Description: "<<e.what());
    res.answer_received = false;
    return false;
  }
  return true;
}

bool getInfoAllGroups(pses_basis::GetInfoAllGroups::Request& req,
                   pses_basis::GetInfoAllGroups::Response& res, Communication* com)
{
  res.answer_received = false;
  std::string cmd = "All Groups Info";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try{
    res.answer_received = com->sendCommand(cmd, input, output);
    if(!res.answer_received) return false;
    output.getParameterValue("info",res.info);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Service 'All Groups Info' occured!\n Description: "<<e.what());
    res.answer_received = false;
    return false;
  }
  return true;
}

bool getInfoGroup(pses_basis::GetInfoGroup::Request& req,
                   pses_basis::GetInfoGroup::Response& res, Communication* com)
{
  res.answer_received = false;
  std::string cmd = "Group Info";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  input.insertParameter("grp_nr", "uint8_t",req.group_number);
  try{
    res.answer_received = com->sendCommand(cmd, input, output);
    if(!res.answer_received) return false;
    output.getParameterValue("info",res.info);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Service 'Group Info' occured!\n Description: "<<e.what());
    res.answer_received = false;
    return false;
  }
  return true;
}

bool getKinectStatus(pses_basis::GetKinectStatus::Request& req,
                   pses_basis::GetKinectStatus::Response& res, Communication* com)
{
  res.answer_received = false;
  std::string cmd = "Get Kinect Status";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try{
    res.answer_received = com->sendCommand(cmd, input, output);
    if(!res.answer_received) return false;
    output.getParameterValue("kinect_stat",res.status);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Service 'Get Kinect Status' occured!\n Description: "<<e.what());
    res.answer_received = false;
    return false;
  }
  return true;
}

bool getMagASA(pses_basis::GetMagASA::Request& req,
                   pses_basis::GetMagASA::Response& res, Communication* com)
{
  res.answer_received = false;
  std::string cmd = "Get MAG ASA";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try{
    res.answer_received = com->sendCommand(cmd, input, output);
    if(!res.answer_received) return false;
    output.getParameterValue("asa_mx",res.asa_mx);
    output.getParameterValue("asa_my",res.asa_my);
    output.getParameterValue("asa_mz",res.asa_mz);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Service 'Get Mag ASA' occured!\n Description: "<<e.what());
    res.answer_received = false;
    return false;
  }
  return true;
}

bool getMagInfo(pses_basis::GetMagInfo::Request& req,
                   pses_basis::GetMagInfo::Response& res, Communication* com)
{
  res.answer_received = false;
  std::string cmd = "Get MAG Info";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try{
    res.answer_received = com->sendCommand(cmd, input, output);
    if(!res.answer_received) return false;
    output.getParameterValue("info",res.info);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Service 'Get Mag info' occured!\n Description: "<<e.what());
    res.answer_received = false;
    return false;
  }
  return true;
}

bool getMotorLevel(pses_basis::GetMotorLevel::Request& req,
                   pses_basis::GetMotorLevel::Response& res, Communication* com)
{
  res.answer_received = false;
  std::string cmd = "Get Motor Level";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try{
    res.answer_received = com->sendCommand(cmd, input, output);
    if(!res.answer_received) return false;
    std::string direction;
    short level;
    output.getParameterValue("direction",direction);
    output.getParameterValue("level",level);
    if(direction.compare("F")!=0){
      level=-level;
    }
    res.level=level;
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Service 'Get Motor Level' occured!\n Description: "<<e.what());
    res.answer_received = false;
    return false;
  }
  return true;
}

bool getSessionID(pses_basis::GetSessionID::Request& req,
                   pses_basis::GetSessionID::Response& res, Communication* com)
{
  res.answer_received = false;
  std::string cmd = "Get Session ID";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try{
    res.answer_received = com->sendCommand(cmd, input, output);
    if(!res.answer_received) return false;
    output.getParameterValue("ID",res.SID);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Service 'Get Session ID' occured!\n Description: "<<e.what());
    res.answer_received = false;
    return false;
  }
  return true;
}

bool getSteeringLevel(pses_basis::GetSteeringLevel::Request& req,
                   pses_basis::GetSteeringLevel::Response& res, Communication* com)
{
  res.answer_received = false;
  std::string cmd = "Get Steering Level";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try{
    res.answer_received = com->sendCommand(cmd, input, output);
    if(!res.answer_received) return false;
    output.getParameterValue("level",res.level);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Service 'Get Steering Level' occured!\n Description: "<<e.what());
    res.answer_received = false;
    return false;
  }
  return true;
}

bool getUSInfo(pses_basis::GetUSInfo::Request& req,
                   pses_basis::GetUSInfo::Response& res, Communication* com)
{
  res.answer_received = false;
  std::string cmd = "Get US Info";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try{
    res.answer_received = com->sendCommand(cmd, input, output);
    if(!res.answer_received) return false;
    output.getParameterValue("info",res.info);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Service 'Get US info' occured!\n Description: "<<e.what());
    res.answer_received = false;
    return false;
  }
  return true;
}

bool resetController(pses_basis::ResetController::Request& req,
                   pses_basis::ResetController::Response& res, Communication* com)
{
  res.was_set = false;
  std::string cmd = "Reset Controller";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try{
    com->sendCommand(cmd, input, output);
    res.was_set = true;
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Service 'Reset Controller' occured!\n Description: "<<e.what());
    res.was_set = false;
    return false;
  }
  return true;
}

bool resetDMS(pses_basis::ResetDMS::Request& req,
                   pses_basis::ResetDMS::Response& res, Communication* com)
{
  res.was_set = false;
  std::string cmd = "Reset DMS";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try{
    res.was_set = com->sendCommand(cmd, input, output);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Service 'toggleDMS' occured!\n Description: "<<e.what());
    res.was_set = false;
    return false;
  }
  return true;
}

bool setIMU(pses_basis::SetIMU::Request& req,
                   pses_basis::SetIMU::Response& res, Communication* com)
{
  res.was_set = false;
  std::string cmd = "Set IMU";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  input.insertParameter("arange", "int16_t", req.accel_range);
  input.insertParameter("afilt", "int16_t", req.accel_filter);
  input.insertParameter("grange", "int16_t", req.gyro_range);
  input.insertParameter("gfilt", "int16_t", req.gyro_filter);
  std::vector<std::string>options;
  options.push_back("IMU_AccelRange");
  options.push_back("IMU_AccelFilter");
  options.push_back("IMU_GyroRange");
  options.push_back("IMU_GyroFilter");
  options.push_back("IMU_si_info");
  try{
    res.was_set = com->sendCommand(cmd, options, input, output);
    output.getParameterValue("info", res.in_si_units);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Service 'Set IMU' occured!\n Description: "<<e.what());
    res.was_set = false;
    return false;
  }
  return true;
}

bool setMag(pses_basis::SetMag::Request& req,
                   pses_basis::SetMag::Response& res, Communication* com)
{
  res.was_set = false;
  std::string cmd = "Set MAG";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  unsigned char use_asa = req.use_asa;
  input.insertParameter("use_asa", "uint8_t", use_asa);
  std::vector<std::string>options;
  options.push_back("MAG_USEASA");
  try{
    res.was_set = com->sendCommand(cmd, options, input, output);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Service 'Set IMU' occured!\n Description: "<<e.what());
    res.was_set = false;
    return false;
  }
  return true;
}

bool setMotorLevel(pses_basis::SetMotorLevel::Request& req,
                   pses_basis::SetMotorLevel::Response& res, Communication* com)
{
  res.was_set = false;
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
  try{
    res.was_set = com->sendCommand(cmd, input, output);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Service 'Set Motor Level' occured!\n Description: "<<e.what());
    res.was_set = false;
    return false;
  }
  return true;
}

bool setSessionID(pses_basis::SetSessionID::Request& req,
                   pses_basis::SetSessionID::Response& res, Communication* com)
{
  res.was_set = false;
  std::string cmd = "Set Session ID";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  input.insertParameter("ID", "int32_t", req.sid);
  try{
    res.was_set = com->sendCommand(cmd, input, output);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Service 'Set Session ID' occured!\n Description: "<<e.what());
    res.was_set = false;
    return false;
  }
  return true;
}

bool setSteeringLevel(pses_basis::SetSteering::Request& req,
                   pses_basis::SetSteering::Response& res, Communication* com)
{
  res.was_set = false;
  std::string cmd = "Set Steering Level";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  input.insertParameter("steering", "int16_t", req.steering);
  try{
    res.was_set = com->sendCommand(cmd, input, output);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Service 'Set Steering Level' occured!\n Description: "<<e.what());
    res.was_set = false;
    return false;
  }
  return true;
}

bool setUS(pses_basis::SetUS::Request& req,
                   pses_basis::SetUS::Response& res, Communication* com)
{
  res.was_set = false;
  std::string cmd = "Set US";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  input.insertParameter("usrange", "int16_t", req.range);
  input.insertParameter("usgain", "int16_t", req.gain);
  std::vector<std::string>options;
  options.push_back("US_Range");
  options.push_back("US_Gain");
  options.push_back("US_si_info");
  try{
    res.was_set = com->sendCommand(cmd, options, input, output);
    output.getParameterValue("info", res.in_si_units);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Service 'Set US' occured!\n Description: "<<e.what());
    res.was_set = false;
    return false;
  }
  return true;
}

bool toggleBrakes(pses_basis::ToggleBrakes::Request& req,
                   pses_basis::ToggleBrakes::Response& res, Communication* com)
{
  res.was_set = false;
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
  try{
    res.was_set = com->sendCommand(cmd, input, output);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Service 'Toggle Brakes' occured!\n Description: "<<e.what());
    res.was_set = false;
    return false;
  }
  return true;
}

bool toggleDAQ(pses_basis::ToggleDAQ::Request& req,
                   pses_basis::ToggleDAQ::Response& res, Communication* com)
{
  res.was_set = false;
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  std::string cmd;
  if(req.DAQ_on){
    cmd = "Start DAQ";
  }else{
    cmd = "Stop DAQ";
  }
  try{
    res.was_set = com->sendCommand(cmd, input, output);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Service 'Toggle DAQ' occured!\n Description: "<<e.what());
    res.was_set = false;
    return false;
  }
  return true;
}


bool toggleDMS(pses_basis::ToggleDMS::Request& req,
                   pses_basis::ToggleDMS::Response& res, Communication* com)
{
  res.was_set = false;
  std::string cmd = "Set Drv";
  unsigned int interval = 0;
  if (req.dms_on)
  {
    interval = req.dms_interval;
  }

  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  std::vector<std::string> options;
  options.push_back("DMS_Interval");
  input.insertParameter("dms_time", "uint32_t", interval);
  try{
    res.was_set = com->sendCommand(cmd, options, input, output);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Service 'toggleDMS' occured!\n Description: "<<e.what());
    res.was_set = false;
    return false;
  }
  return true;
}

bool toggleGroup(pses_basis::ToggleGroup::Request& req,
                   pses_basis::ToggleGroup::Response& res, Communication* com)
{
  res.was_set = false;
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  std::string cmd;
  if(req.group_on){
    cmd = "Activate Group";
  }else{
    cmd = "Deactivate Group";
  }
  input.insertParameter("grp_nr", "uint8_t", req.group_number);
  try{
    res.was_set = com->sendCommand(cmd, input, output);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Service 'Toggle Group' occured!\n Description: "<<e.what());
    res.was_set = false;
    return false;
  }
  return true;
}

bool toggleKinect(pses_basis::ToggleKinect::Request& req,
                   pses_basis::ToggleKinect::Response& res, Communication* com)
{
  res.was_set = false;
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  std::string cmd;
  if(req.kinect_on){
    cmd = "Toggle Kinect On";
  }else{
    cmd = "Toggle Kinect Off";
  }

  try{
    res.was_set = com->sendCommand(cmd, input, output);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Service 'Toggle Kinect' occured!\n Description: "<<e.what());
    res.was_set = false;
    return false;
  }
  return true;
}

bool toggleMotor(pses_basis::ToggleMotor::Request& req,
                   pses_basis::ToggleMotor::Response& res, Communication* com)
{
  res.was_set = false;
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
  try{
    res.was_set = com->sendCommand(cmd, input, output);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Service 'Toggle Motor' occured!\n Description: "<<e.what());
    res.was_set = false;
    return false;
  }
  return true;
}

bool toggleUS(pses_basis::ToggleUS::Request& req,
                   pses_basis::ToggleUS::Response& res, Communication* com)
{
  res.was_set = false;
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  std::string cmd;
  if(req.us_on){
    cmd = "Toggle US On";
  }else{
    cmd = "Toggle US Off";
  }

  try{
    res.was_set = com->sendCommand(cmd, input, output);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Service 'Toggle US' occured!\n Description: "<<e.what());
    res.was_set = false;
    return false;
  }
  return true;
}

}
#endif // SERVICEFUNCTIONS_H
