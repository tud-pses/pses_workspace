#include <ros/ros.h>
#include <signal.h>
#include <pses_basis/servicefunctions.h>
#include <pses_basis/communication.h>
#include <pses_basis/communicationconfig.h>
#include <ros/package.h>
#include <std_msgs/builtin_uint8.h>
#include <std_msgs/builtin_uint16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/BatteryState.h>

void publishSensorGroupMessage1(
    SensorGroup* grp, std::unordered_map<std::string, ros::Publisher*>& pub)
{
  // do stuff
  // ROS_INFO_STREAM("doing stuff");
  sensor_msgs::Range usl, usr, usf;
  //unsigned short l, r, f;
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
    //usl.range = l;
    usl.max_range = 3;
    usl.min_range = 0.06;
    usl.field_of_view = 0.76;
    usl.radiation_type = 0;
    usl.header.frame_id = "robot_left_us";
    usl.header.stamp = t;

    //usf.range = f;
    usf.max_range = 3;
    usf.min_range = 0.06;
    usf.field_of_view = 0.76;
    usf.radiation_type = 0;
    usf.header.frame_id = "robot_front_us";
    usf.header.stamp = t;

    //usr.range = r;
    usr.max_range = 3;
    usr.min_range = 0.06;
    usr.field_of_view = 0.76;
    usr.radiation_type = 0;
    usr.header.frame_id = "robot_right_us";
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
  // do stuff
  // ROS_INFO_STREAM("doing stuff");
  sensor_msgs::Imu imu;
  //short gx, gy, gz, ax, ay, az;
  ros::Time t = ros::Time::now();
  try
  {
    grp->getChannelValueConverted("GX", imu.angular_velocity.x);
    grp->getChannelValueConverted("GY", imu.angular_velocity.y);
    grp->getChannelValueConverted("GZ", imu.angular_velocity.z);
    grp->getChannelValueConverted("AX", imu.linear_acceleration.x);
    grp->getChannelValueConverted("AY", imu.linear_acceleration.y);
    grp->getChannelValueConverted("AZ", imu.linear_acceleration.z);
    //imu.angular_velocity.x = gx;
    //imu.angular_velocity.y = gy;
    //imu.angular_velocity.z = gz;
    //imu.linear_acceleration.x = ax;
    //imu.linear_acceleration.y = ay;
    //imu.linear_acceleration.z = az;
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
    //mag.magnetic_field.x = mx;
    //mag.magnetic_field.y = my;
    //mag.magnetic_field.z = mz;
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
  //unsigned short vsbat, vdbat;
  double vsbat, vdbat;
  ros::Time t = ros::Time::now();
  try
  {
    grp->getChannelValueConverted("VDBAT", vdbat);
    grp->getChannelValueConverted("VSBAT", vsbat);
    batVD.voltage = vdbat;
    batVS.voltage = vsbat;
    //batVD.voltage = vdbat;
    //batVS.voltage = vsbat;
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

void publishDebugMessage(
    const std::string& msg, ros::Publisher* pub)
{
  std_msgs::String debug;
  debug.data = msg;
  pub->publish(debug);
}


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
  ros::Time t = ros::Time::now();
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
  ROS_INFO_STREAM("Round trip t: "<<(ros::Time::now()-t).toSec());
}

void ucBoardMessageCallback(std_msgs::String::ConstPtr msg, Communication* com){
  try{
    com->sendRawMessage(msg->data);
  }catch(std::exception& e){
    ROS_WARN_STREAM("An error in Message 'send_uc_board_msg' occured!\n Description: "<<e.what());
  }
}

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
}

int main(int argc, char** argv)
{
  // set up ros node handle
  ros::init(argc, argv, "uc_bridge", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  // get launch paramer
  bool regSensorGrps, motorCtrlOn, steeringCtrlOn, kinectOn, usOn,
      daqOn, debugMsgOn, rawComOn;
  std::string configPath;
  nh.getParam("/uc_bridge/reset_on_shutdown", uc_bridge::rstOnShutdown);
  nh.getParam("/uc_bridge/register_sensor_groups", regSensorGrps);
  nh.getParam("/uc_bridge/activate_motor_controller", motorCtrlOn);
  nh.getParam("/uc_bridge/activate_steering_controller", steeringCtrlOn);
  nh.getParam("/uc_bridge/activate_kinect", kinectOn);
  nh.getParam("/uc_bridge/activate_us_sensors", usOn);
  nh.getParam("/uc_bridge/activate_daq", daqOn);
  nh.getParam("/uc_bridge/config_path", configPath);
  nh.getParam("/uc_bridge/enable_debug_messages", debugMsgOn);
  nh.getParam("/uc_bridge/enable_raw_communiction", rawComOn);
  if(rawComOn){
    regSensorGrps = false;
    motorCtrlOn = false;
    steeringCtrlOn = false;
    kinectOn = false;
    usOn = false;
    daqOn = false;
  }
  // load communication config files and init communication
  //std::string typesPath = ros::package::getPath("pses_basis") + "/config/";
  Communication com(configPath);
  uc_bridge::com_ptr = &com;
  // create sensor group publish services
  ros::Publisher grp11 = nh.advertise<sensor_msgs::Range>("USL", 10);
  ros::Publisher grp12 = nh.advertise<sensor_msgs::Range>("USF", 10);
  ros::Publisher grp13 = nh.advertise<sensor_msgs::Range>("USR", 10);
  ros::Publisher grp2 = nh.advertise<sensor_msgs::Imu>("IMU", 10);
  ros::Publisher grp31 = nh.advertise<std_msgs::UInt8>("HALL_CNT", 10);
  ros::Publisher grp32 = nh.advertise<std_msgs::Float64>("HALL_DT", 10);
  ros::Publisher grp33 = nh.advertise<std_msgs::Float64>("HALL_DT8", 10);
  ros::Publisher grp4 = nh.advertise<sensor_msgs::MagneticField>("MAG", 10);
  ros::Publisher grp51 = nh.advertise<sensor_msgs::BatteryState>("VDBAT", 10);
  ros::Publisher grp52 = nh.advertise<sensor_msgs::BatteryState>("VSBAT", 10);
  ros::Publisher debug;
  if(debugMsgOn) debug = nh.advertise<std_msgs::String>("DEBUG", 10);
  // group publish services by putting them into a map
  std::unordered_map<std::string, ros::Publisher*> usGrp;
  std::unordered_map<std::string, ros::Publisher*> imuGrp;
  std::unordered_map<std::string, ros::Publisher*> hallGrp;
  std::unordered_map<std::string, ros::Publisher*> magGrp;
  std::unordered_map<std::string, ros::Publisher*> batGrp;
  usGrp.insert(std::make_pair("USL", &grp11));
  usGrp.insert(std::make_pair("USF", &grp12));
  usGrp.insert(std::make_pair("USR", &grp13));
  imuGrp.insert(std::make_pair("IMU", &grp2));
  hallGrp.insert(std::make_pair("HALL_CNT", &grp31));
  hallGrp.insert(std::make_pair("HALL_DT", &grp32));
  hallGrp.insert(std::make_pair("HALL_DT8", &grp33));
  magGrp.insert(std::make_pair("MAG", &grp4));
  batGrp.insert(std::make_pair("VDBAT", &grp51));
  batGrp.insert(std::make_pair("VSBAT", &grp52));

  // register publish callbacks with the uc-board communication framework
  if(regSensorGrps){
    com.registerSensorGroupCallback(
        1, boost::bind(&publishSensorGroupMessage1, _1, usGrp));
    com.registerSensorGroupCallback(
        2, boost::bind(&publishSensorGroupMessage2, _1, imuGrp));
    com.registerSensorGroupCallback(
        3, boost::bind(&publishSensorGroupMessage3, _1, hallGrp));
    com.registerSensorGroupCallback(
        4, boost::bind(&publishSensorGroupMessage4, _1, magGrp));
    com.registerSensorGroupCallback(
        5, boost::bind(&publishSensorGroupMessage5, _1, batGrp));
  }
  if(debugMsgOn) com.enableDebugMessages(boost::bind(&publishDebugMessage, _1, &debug));
  if(rawComOn) com.enableRawCommunication();

  // start serial communication
  try
  {
    com.connect();
    com.startCommunication();
    //ros::Duration(1.0).sleep();
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM("An error occured while trying to set up the connection.\n "
                    "Description: "
                    << e.what());
    uc_bridge::shutdownSignalHandler(0);
  }

  if(regSensorGrps){
    uc_bridge::registerSensorGroups(&com);
  }

  if(motorCtrlOn){
    uc_bridge::activateMotorController(&com);
  }

  if(steeringCtrlOn){
    uc_bridge::activateSteeringController(&com);
  }

  if(kinectOn){
    uc_bridge::activateKinect(&com);
  }

  if(usOn){
    uc_bridge::activateUS(&com);
  }

  if(daqOn){
    uc_bridge::activateDAQ(&com);
  }

  // register uc-board communication services with ros
  ros::ServiceServer deleteGroupService =
      nh.advertiseService<pses_basis::DeleteGroup::Request,
                          pses_basis::DeleteGroup::Response>(
          "delete_group",
          std::bind(ServiceFunctions::deleteGroup, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer getControllerIDService =
      nh.advertiseService<pses_basis::GetControllerID::Request,
                          pses_basis::GetControllerID::Response>(
          "get_controller_id",
          std::bind(ServiceFunctions::getControllerID, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer getDAQStatusService =
      nh.advertiseService<pses_basis::GetDAQStatus::Request,
                          pses_basis::GetDAQStatus::Response>(
          "get_daq_status",
          std::bind(ServiceFunctions::getDAQStatus, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer getFirmwareVersionService =
      nh.advertiseService<pses_basis::GetFirmwareVersion::Request,
                          pses_basis::GetFirmwareVersion::Response>(
          "get_firmware_version",
          std::bind(ServiceFunctions::getFirmwareVersion, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer getInfoAllGroupsService =
      nh.advertiseService<pses_basis::GetInfoAllGroups::Request,
                          pses_basis::GetInfoAllGroups::Response>(
          "get_info_all_groups",
          std::bind(ServiceFunctions::getInfoAllGroups, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer getInfoGroupService =
      nh.advertiseService<pses_basis::GetInfoGroup::Request,
                          pses_basis::GetInfoGroup::Response>(
          "get_info_group",
          std::bind(ServiceFunctions::getInfoGroup, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer getKinectStatusService =
      nh.advertiseService<pses_basis::GetKinectStatus::Request,
                          pses_basis::GetKinectStatus::Response>(
          "get_kinect_status",
          std::bind(ServiceFunctions::getKinectStatus, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer getMotorLevelService =
      nh.advertiseService<pses_basis::GetMotorLevel::Request,
                          pses_basis::GetMotorLevel::Response>(
          "get_motor_level",
          std::bind(ServiceFunctions::getMotorLevel, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer getSessionIDService =
      nh.advertiseService<pses_basis::GetSessionID::Request,
                          pses_basis::GetSessionID::Response>(
          "get_session_id",
          std::bind(ServiceFunctions::getSessionID, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer getSteeringLevelService =
      nh.advertiseService<pses_basis::GetSteeringLevel::Request,
                          pses_basis::GetSteeringLevel::Response>(
          "get_steering_level",
          std::bind(ServiceFunctions::getSteeringLevel, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer resetControllerService =
      nh.advertiseService<pses_basis::ResetController::Request,
                          pses_basis::ResetController::Response>(
          "reset_controller",
          std::bind(ServiceFunctions::resetController, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer setMotorLevelService =
      nh.advertiseService<pses_basis::SetMotorLevel::Request,
                          pses_basis::SetMotorLevel::Response>(
          "set_motor_level",
          std::bind(ServiceFunctions::setMotorLevel, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer setSessionIDService =
      nh.advertiseService<pses_basis::SetSessionID::Request,
                          pses_basis::SetSessionID::Response>(
          "set_session_id",
          std::bind(ServiceFunctions::setSessionID, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer setSteeringLevelService =
      nh.advertiseService<pses_basis::SetSteering::Request,
                          pses_basis::SetSteering::Response>(
          "set_steering_level",
          std::bind(ServiceFunctions::setSteeringLevel, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer toggleBrakesService =
      nh.advertiseService<pses_basis::ToggleBrakes::Request,
                          pses_basis::ToggleBrakes::Response>(
          "toggle_brakes",
          std::bind(ServiceFunctions::toggleBrakes, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer toggleDAQService =
      nh.advertiseService<pses_basis::ToggleDAQ::Request,
                          pses_basis::ToggleDAQ::Response>(
          "toggle_daq",
          std::bind(ServiceFunctions::toggleDAQ, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer toggleGroupService =
      nh.advertiseService<pses_basis::ToggleGroup::Request,
                          pses_basis::ToggleGroup::Response>(
          "toggle_group",
          std::bind(ServiceFunctions::toggleGroup, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer toggleKinectService =
      nh.advertiseService<pses_basis::ToggleKinect::Request,
                          pses_basis::ToggleKinect::Response>(
          "toggle_kinect",
          std::bind(ServiceFunctions::toggleKinect, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer toggleMotorService =
      nh.advertiseService<pses_basis::ToggleMotor::Request,
                          pses_basis::ToggleMotor::Response>(
          "toggle_motor",
          std::bind(ServiceFunctions::toggleMotor, std::placeholders::_1,
                    std::placeholders::_2, &com));
  ros::ServiceServer toggleUSService =
      nh.advertiseService<pses_basis::ToggleUS::Request,
                          pses_basis::ToggleUS::Response>(
          "toggle_us",
          std::bind(ServiceFunctions::toggleUS, std::placeholders::_1,
                    std::placeholders::_2, &com));

  // create control subscribers e.g. steering, motorlevel etc.
  ros::Subscriber motorLevelSubscriber = nh.subscribe<std_msgs::Int16>("set_motor_level_msg",10, boost::bind(motorLevelCallback, _1, &com));
  ros::Subscriber steeringLevelSubscriber = nh.subscribe<std_msgs::Int16>("set_steering_level_msg",10, boost::bind(steeringLevelCallback, _1 , &com));
  ros::Subscriber ucBoardMsgSubscriber;
  if(rawComOn) ucBoardMsgSubscriber = nh.subscribe<std_msgs::String>("send_uc_board_msg",10, boost::bind(ucBoardMessageCallback, _1 , &com));


  // register shut down signal handler
  signal(SIGINT, uc_bridge::shutdownSignalHandler);

  ros::spin();
  return 0;
}
