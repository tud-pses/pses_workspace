#ifndef UC_BRIDGE_H
#define UC_BRIDGE_H

#include <ros/ros.h>
#include <signal.h>
#include <pses_ucbridge/servicefunctions.h>
#include <pses_ucbridge/communication.h>
#include <pses_ucbridge/communicationconfig.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/BatteryState.h>

namespace uc_bridge
{
Communication* com_ptr;
bool rstOnShutdown;

void resetController(Communication* com)
{
  std::string cmd = "Reset Controller";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  try
  {
    com->sendCommand(cmd, input, output);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error occured during initial Reset!\n Description: " << e.what());
  }
}

void shutdownSignalHandler(int sig)
{
  if(rstOnShutdown){
    resetController(com_ptr);
    ros::Duration(0.1).sleep();
  }
  try
  {
    com_ptr->stopCommunication();
    com_ptr->disconnect();
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM("An error occured while trying to shut down the "
                    "connection.\n Description: "
                    << e.what());
  }
  ros::shutdown();
}

void registerSensorGroups(Communication* com)
{
  // register sensor groups
  try
  {
    if (!com->registerSensorGroups("Set Group"))
      ROS_WARN_STREAM("Registering all sensor groups failed!");
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM("An error occured while trying to register sensor "
                    "groups!\n Description: "
                    << e.what());
  }
}

void activateMotorController(Communication* com)
{
  bool was_set = false;
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  std::string cmd = "Drive Forward";
  short level = 0;
  input.insertParameter("speed", "int16_t", level);
  try
  {
    was_set = com->sendCommand(cmd, input, output);
    if (!was_set)
      ROS_WARN_STREAM("Activating motor controller failed!");
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM("An error occured while trying to activate the motor "
                    "controller!\n Description: "
                    << e.what());
  }
}

void activateSteeringController(Communication* com)
{
  bool was_set = false;
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  std::string cmd = "Set Steering Level";
  short level = 0;
  input.insertParameter("steering", "int16_t", level);
  try
  {
    was_set = com->sendCommand(cmd, input, output);
    if (!was_set)
      ROS_WARN_STREAM("Activating steering controller failed!");
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM("An error occured while trying to activate the steering "
                    "controller!\n Description: "
                    << e.what());
  }
}

void activateKinect(Communication* com)
{
  bool was_set = false;
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  std::string cmd = "Toggle Kinect On";
  try
  {
    was_set = com->sendCommand(cmd, input, output);
    if (!was_set)
      ROS_WARN_STREAM("Activating kinect failed!");
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error occured while trying to activate kinect!\n Description: "
        << e.what());
  }
}

void activateUS(Communication* com)
{
  bool was_set = false;
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  std::string cmd = "Toggle US On";
  try
  {
    was_set = com->sendCommand(cmd, input, output);
    if (!was_set)
      ROS_WARN_STREAM("Activating us-sensors failed!");
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error occured while trying to activate us-sensors!\n Description: "
        << e.what());
  }
}

void activateDAQ(Communication* com)
{
  bool was_set = false;
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  std::string cmd = "Start DAQ";
  try
  {
    was_set = com->sendCommand(cmd, input, output);
    if (!was_set)
      ROS_WARN_STREAM("Activating daq failed!");
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM(
        "An error occured while trying to activate daq!\n Description: "
        << e.what());
  }
}

// sensor group callbacks
void publishSensorGroupMessage1(
    SensorGroup* grp, std::unordered_map<std::string, ros::Publisher*>& pub)
{
  sensor_msgs::Range usl, usr, usf;
  double l, r, f;
  ros::Time t = ros::Time::now();
  try
  {
    grp->getChannelValueConverted("USL", l);
    grp->getChannelValueConverted("USF", f);
    grp->getChannelValueConverted("USR", r);
    usl.range = l;
    usf.range = f;
    usr.range = r;

    usl.max_range = 3;
    usl.min_range = 0.06;
    usl.field_of_view = 0.76;
    usl.radiation_type = 0;
    usl.header.frame_id = "left_sensor";
    usl.header.stamp = t;

    usf.max_range = 3;
    usf.min_range = 0.06;
    usf.field_of_view = 0.76;
    usf.radiation_type = 0;
    usf.header.frame_id = "front_sensor";
    usf.header.stamp = t;

    usr.max_range = 3;
    usr.min_range = 0.06;
    usr.field_of_view = 0.76;
    usr.radiation_type = 0;
    usr.header.frame_id = "right_sensor";
    usr.header.stamp = t;

    pub["USL"]->publish(usl);
    pub["USF"]->publish(usf);
    pub["USR"]->publish(usr);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM("An error in Message 'Sensor grp1' occured!\n Description: "
                    << e.what());
  }
}

void publishSensorGroupMessage2(
    SensorGroup* grp, std::unordered_map<std::string, ros::Publisher*>& pub)
{
  sensor_msgs::Imu imu;
  ros::Time t = ros::Time::now();
  try
  {
    grp->getChannelValueConverted("GX", imu.angular_velocity.x);
    grp->getChannelValueConverted("GY", imu.angular_velocity.y);
    grp->getChannelValueConverted("GZ", imu.angular_velocity.z);
    grp->getChannelValueConverted("AX", imu.linear_acceleration.x);
    grp->getChannelValueConverted("AY", imu.linear_acceleration.y);
    grp->getChannelValueConverted("AZ", imu.linear_acceleration.z);
    imu.header.stamp = t;
    imu.header.frame_id = "robot_imu";

    pub["IMU"]->publish(imu);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM("An error in Message 'Sensor grp2' occured!\n Description: "
                    << e.what());
  }
}

void publishSensorGroupMessage3(
    SensorGroup* grp, std::unordered_map<std::string, ros::Publisher*>& pub)
{
  std_msgs::UInt8 hallcnt;
  std_msgs::Float64 halldt, halldt8;
  try
  {
    grp->getChannelValue("HALL_CNT", hallcnt.data);
    grp->getChannelValueConverted("HALL_DT", halldt.data);
    grp->getChannelValueConverted("HALL_DT8", halldt8.data);
    pub["HALL_CNT"]->publish(hallcnt);
    pub["HALL_DT"]->publish(halldt);
    pub["HALL_DT8"]->publish(halldt8);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM("An error in Message 'Sensor grp3' occured!\n Description: "
                    << e.what());
  }
}

void publishSensorGroupMessage4(
    SensorGroup* grp, std::unordered_map<std::string, ros::Publisher*>& pub)
{
  sensor_msgs::MagneticField mag;
  //short mx, my, mz;
  ros::Time t = ros::Time::now();
  try
  {
    grp->getChannelValueConverted("MX", mag.magnetic_field.x);
    grp->getChannelValueConverted("MY", mag.magnetic_field.y);
    grp->getChannelValueConverted("MZ", mag.magnetic_field.z);
    mag.header.stamp = t;
    mag.header.frame_id = "robot_magnetometer";
    pub["MAG"]->publish(mag);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM("An error in Message 'Sensor grp4' occured!\n Description: "
                    << e.what());
  }
}

void publishSensorGroupMessage5(
    SensorGroup* grp, std::unordered_map<std::string, ros::Publisher*>& pub)
{
  sensor_msgs::BatteryState batVD, batVS;
  double vsbat, vdbat;
  ros::Time t = ros::Time::now();
  try
  {
    grp->getChannelValueConverted("VDBAT", vdbat);
    grp->getChannelValueConverted("VSBAT", vsbat);
    batVD.voltage = vdbat;
    batVS.voltage = vsbat;
    batVD.header.stamp = t;
    batVS.header.stamp = t;

    pub["VDBAT"]->publish(batVD);
    pub["VSBAT"]->publish(batVS);
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM("An error in Message 'Sensor grp5' occured!\n Description: "
                    << e.what());
  }
}

// debug/error/text callbacks
void errorCallback(
    const std::string& msg)
{
  ROS_WARN_STREAM("A communication Error occured!\n"<<msg);
}

void textCallback(
    const std::string& msg)
{
  ROS_INFO_STREAM("UC board sent the following info:\n"<<msg);
}

void publishDebugMessage(
    const std::string& msg, ros::Publisher* pub)
{
  std_msgs::String debug;
  debug.data = msg;
  pub->publish(debug);
}

// ros command message callbacks
void motorLevelCallback(std_msgs::Int16::ConstPtr motorLevel, Communication* com){
  bool was_set = false;
  std::string cmd = "Drive Forward";
  short level = motorLevel->data;
  if (level < 0)
  {
    cmd = "Drive Backward";
    level = -level;
  }
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  input.insertParameter("speed", "int16_t", level);
  try{
    was_set = com->sendCommand(cmd, input, output);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Message 'set_motor_level_msg' occured!\n Description: "<<e.what());
    was_set = false;
  }
  if(!was_set) ROS_WARN_STREAM("Motor level set to: "<<motorLevel->data<<" failed!");
}

void steeringLevelCallback(std_msgs::Int16::ConstPtr steeringLevel, Communication* com){
  bool was_set = false;
  std::string cmd = "Set Steering Level";
  Parameter::ParameterMap input;
  Parameter::ParameterMap output;
  input.insertParameter("steering", "int16_t", steeringLevel->data);
  try{
    was_set = com->sendCommand(cmd, input, output);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Message 'set_steering_level_msg' occured!\n Description: "<<e.what());
    was_set = false;
  }
  if(!was_set) ROS_WARN_STREAM("Steering level set to: "<<steeringLevel->data<<" failed!");
}

// debug raw message callback
void ucBoardMessageCallback(std_msgs::String::ConstPtr msg, Communication* com){
  try{
    com->sendRawMessage(msg->data);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Message 'send_uc_board_msg' occured!\n Description: "<<e.what());
  }
}
}

#endif // UC_BRIDGE_H
