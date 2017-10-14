#include <pses_ucbridge/uc_bridge.h>

int main(int argc, char** argv)
{
  // set up ros node handle
  ros::init(argc, argv, "uc_bridge", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  // get launch paramer
  bool regSensorGrps, motorCtrlOn, steeringCtrlOn, kinectOn, usOn, daqOn,
      debugMsgOn, rawComOn;
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
  if (rawComOn)
  {
    regSensorGrps = false;
    motorCtrlOn = false;
    steeringCtrlOn = false;
    kinectOn = false;
    usOn = false;
    daqOn = false;
  }
  // load communication config files and init communication
  // std::string typesPath = ros::package::getPath("pses_ucbridge") + "/config/";
  Communication com(configPath);
  uc_bridge::com_ptr = &com;
  // create sensor group publish services
  ros::Publisher grp11 = nh.advertise<sensor_msgs::Range>("/uc_bridge/usl", 10);
  ros::Publisher grp12 = nh.advertise<sensor_msgs::Range>("/uc_bridge/usf", 10);
  ros::Publisher grp13 = nh.advertise<sensor_msgs::Range>("/uc_bridge/usr", 10);
  ros::Publisher grp2 = nh.advertise<sensor_msgs::Imu>("/uc_bridge/imu", 10);
  ros::Publisher grp31 =
      nh.advertise<std_msgs::UInt8>("/uc_bridge/hall_cnt", 10);
  ros::Publisher grp32 =
      nh.advertise<std_msgs::Float64>("/uc_bridge/hall_dt", 10);
  ros::Publisher grp33 =
      nh.advertise<std_msgs::Float64>("/uc_bridge/hall_dt8", 10);
  ros::Publisher grp4 =
      nh.advertise<sensor_msgs::MagneticField>("/uc_bridge/mag", 10);
  ros::Publisher grp51 =
      nh.advertise<sensor_msgs::BatteryState>("/uc_bridge/vdbat", 10);
  ros::Publisher grp52 =
      nh.advertise<sensor_msgs::BatteryState>("/uc_bridge/vsbat", 10);
  ros::Publisher debug;
  if (debugMsgOn)
    debug = nh.advertise<std_msgs::String>("DEBUG", 10);
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
  if (regSensorGrps)
  {
    com.registerSensorGroupCallback(
        1, boost::bind(&uc_bridge::publishSensorGroupMessage1, _1, usGrp));
    com.registerSensorGroupCallback(
        2, boost::bind(&uc_bridge::publishSensorGroupMessage2, _1, imuGrp));
    com.registerSensorGroupCallback(
        3, boost::bind(&uc_bridge::publishSensorGroupMessage3, _1, hallGrp));
    com.registerSensorGroupCallback(
        4, boost::bind(&uc_bridge::publishSensorGroupMessage4, _1, magGrp));
    com.registerSensorGroupCallback(
        5, boost::bind(&uc_bridge::publishSensorGroupMessage5, _1, batGrp));
  }
  // debug special modes
  if (debugMsgOn)
    com.enableDebugMessages(
        boost::bind(&uc_bridge::publishDebugMessage, _1, &debug));
  if (rawComOn)
    com.enableRawCommunication();

  // register error/txt-message callbacks
  com.registerErrorCallback(&uc_bridge::errorCallback);
  com.registerTextCallback(&uc_bridge::textCallback);

  // start serial communication
  try
  {
    com.connect();
    com.startCommunication();
  }
  catch (std::exception& e)
  {
    ROS_WARN_STREAM("An error occured while trying to set up the connection.\n "
                    "Description: "
                    << e.what());
    uc_bridge::shutdownSignalHandler(0);
  }

  // configure communication according to start up parameters
  if (regSensorGrps)
  {
    uc_bridge::registerSensorGroups(&com);
  }

  if (motorCtrlOn)
  {
    uc_bridge::activateMotorController(&com);
  }

  if (steeringCtrlOn)
  {
    uc_bridge::activateSteeringController(&com);
  }

  if (kinectOn)
  {
    uc_bridge::activateKinect(&com);
  }

  if (usOn)
  {
    uc_bridge::activateUS(&com);
  }

  if (daqOn)
  {
    uc_bridge::activateDAQ(&com);
  }

  // register uc-board communication services with ros
  ros::ServiceServer deleteGroupService =
      nh.advertiseService<pses_ucbridge::DeleteGroup::Request,
                          pses_ucbridge::DeleteGroup::Response>(
          "/uc_bridge/delete_group",
          std::bind(ServiceFunctions::deleteGroup, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer getControllerIDService =
      nh.advertiseService<pses_ucbridge::GetControllerID::Request,
                          pses_ucbridge::GetControllerID::Response>(
          "/uc_bridge/get_controller_id",
          std::bind(ServiceFunctions::getControllerID, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer getDAQStatusService =
      nh.advertiseService<pses_ucbridge::GetDAQStatus::Request,
                          pses_ucbridge::GetDAQStatus::Response>(
          "/uc_bridge/get_daq_status",
          std::bind(ServiceFunctions::getDAQStatus, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer getFirmwareVersionService =
      nh.advertiseService<pses_ucbridge::GetFirmwareVersion::Request,
                          pses_ucbridge::GetFirmwareVersion::Response>(
          "/uc_bridge/get_firmware_version",
          std::bind(ServiceFunctions::getFirmwareVersion, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer getIMUInfoService =
      nh.advertiseService<pses_ucbridge::GetIMUInfo::Request,
                          pses_ucbridge::GetIMUInfo::Response>(
          "/uc_bridge/get_imu_info",
          std::bind(ServiceFunctions::getIMUInfo, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer getInfoAllGroupsService =
      nh.advertiseService<pses_ucbridge::GetInfoAllGroups::Request,
                          pses_ucbridge::GetInfoAllGroups::Response>(
          "/uc_bridge/get_info_all_groups",
          std::bind(ServiceFunctions::getInfoAllGroups, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer getInfoGroupService =
      nh.advertiseService<pses_ucbridge::GetInfoGroup::Request,
                          pses_ucbridge::GetInfoGroup::Response>(
          "/uc_bridge/get_info_group",
          std::bind(ServiceFunctions::getInfoGroup, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer getKinectStatusService =
      nh.advertiseService<pses_ucbridge::GetKinectStatus::Request,
                          pses_ucbridge::GetKinectStatus::Response>(
          "/uc_bridge/get_kinect_status",
          std::bind(ServiceFunctions::getKinectStatus, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer getMagASAService =
      nh.advertiseService<pses_ucbridge::GetMagASA::Request,
                          pses_ucbridge::GetMagASA::Response>(
          "/uc_bridge/get_mag_asa",
          std::bind(ServiceFunctions::getMagASA, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer getMagInfoService =
      nh.advertiseService<pses_ucbridge::GetMagInfo::Request,
                          pses_ucbridge::GetMagInfo::Response>(
          "/uc_bridge/get_mag_info",
          std::bind(ServiceFunctions::getMagInfo, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer getMotorLevelService =
      nh.advertiseService<pses_ucbridge::GetMotorLevel::Request,
                          pses_ucbridge::GetMotorLevel::Response>(
          "/uc_bridge/get_motor_level",
          std::bind(ServiceFunctions::getMotorLevel, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer getSessionIDService =
      nh.advertiseService<pses_ucbridge::GetSessionID::Request,
                          pses_ucbridge::GetSessionID::Response>(
          "/uc_bridge/get_session_id",
          std::bind(ServiceFunctions::getSessionID, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer getSteeringLevelService =
      nh.advertiseService<pses_ucbridge::GetSteeringLevel::Request,
                          pses_ucbridge::GetSteeringLevel::Response>(
          "/uc_bridge/get_steering_level",
          std::bind(ServiceFunctions::getSteeringLevel, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer getUSInfoService =
      nh.advertiseService<pses_ucbridge::GetUSInfo::Request,
                          pses_ucbridge::GetUSInfo::Response>(
          "/uc_bridge/get_us_info",
          std::bind(ServiceFunctions::getUSInfo, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer resetControllerService =
      nh.advertiseService<pses_ucbridge::ResetController::Request,
                          pses_ucbridge::ResetController::Response>(
          "/uc_bridge/reset_controller",
          std::bind(ServiceFunctions::resetController, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer resetDMSService =
      nh.advertiseService<pses_ucbridge::ResetDMS::Request,
                          pses_ucbridge::ResetDMS::Response>(
          "/uc_bridge/reset_dms",
          std::bind(ServiceFunctions::resetDMS, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer setIMUService =
      nh.advertiseService<pses_ucbridge::SetIMU::Request,
                          pses_ucbridge::SetIMU::Response>(
          "/uc_bridge/set_imu",
          std::bind(ServiceFunctions::setIMU, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer setMagService =
      nh.advertiseService<pses_ucbridge::SetMag::Request,
                          pses_ucbridge::SetMag::Response>(
          "/uc_bridge/set_mag",
          std::bind(ServiceFunctions::setMag, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer setMotorLevelService =
      nh.advertiseService<pses_ucbridge::SetMotorLevel::Request,
                          pses_ucbridge::SetMotorLevel::Response>(
          "/uc_bridge/set_motor_level",
          std::bind(ServiceFunctions::setMotorLevel, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer setSessionIDService =
      nh.advertiseService<pses_ucbridge::SetSessionID::Request,
                          pses_ucbridge::SetSessionID::Response>(
          "/uc_bridge/set_session_id",
          std::bind(ServiceFunctions::setSessionID, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer setSteeringLevelService =
      nh.advertiseService<pses_ucbridge::SetSteering::Request,
                          pses_ucbridge::SetSteering::Response>(
          "/uc_bridge/set_steering_level",
          std::bind(ServiceFunctions::setSteeringLevel, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer setUSService =
      nh.advertiseService<pses_ucbridge::SetUS::Request,
                          pses_ucbridge::SetUS::Response>(
          "/uc_bridge/set_us",
          std::bind(ServiceFunctions::setUS, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer toggleBrakesService =
      nh.advertiseService<pses_ucbridge::ToggleBrakes::Request,
                          pses_ucbridge::ToggleBrakes::Response>(
          "/uc_bridge/toggle_brakes",
          std::bind(ServiceFunctions::toggleBrakes, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer toggleDAQService =
      nh.advertiseService<pses_ucbridge::ToggleDAQ::Request,
                          pses_ucbridge::ToggleDAQ::Response>(
          "/uc_bridge/toggle_daq",
          std::bind(ServiceFunctions::toggleDAQ, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer toggleDMSService =
      nh.advertiseService<pses_ucbridge::ToggleDMS::Request,
                          pses_ucbridge::ToggleDMS::Response>(
          "/uc_bridge/toggle_dms",
          std::bind(ServiceFunctions::toggleDMS, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer toggleGroupService =
      nh.advertiseService<pses_ucbridge::ToggleGroup::Request,
                          pses_ucbridge::ToggleGroup::Response>(
          "/uc_bridge/toggle_group",
          std::bind(ServiceFunctions::toggleGroup, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer toggleKinectService =
      nh.advertiseService<pses_ucbridge::ToggleKinect::Request,
                          pses_ucbridge::ToggleKinect::Response>(
          "/uc_bridge/toggle_kinect",
          std::bind(ServiceFunctions::toggleKinect, std::placeholders::_1,
                    std::placeholders::_2, &com));

  ros::ServiceServer toggleMotorService =
      nh.advertiseService<pses_ucbridge::ToggleMotor::Request,
                          pses_ucbridge::ToggleMotor::Response>(
          "/uc_bridge/toggle_motor",
          std::bind(ServiceFunctions::toggleMotor, std::placeholders::_1,
                    std::placeholders::_2, &com));
  ros::ServiceServer toggleUSService =
      nh.advertiseService<pses_ucbridge::ToggleUS::Request,
                          pses_ucbridge::ToggleUS::Response>(
          "/uc_bridge/toggle_us",
          std::bind(ServiceFunctions::toggleUS, std::placeholders::_1,
                    std::placeholders::_2, &com));

  // create control subscribers e.g. steering, motorlevel etc.
  ros::Subscriber motorLevelSubscriber = nh.subscribe<std_msgs::Int16>(
      "/uc_bridge/set_motor_level_msg", 1,
      boost::bind(uc_bridge::motorLevelCallback, _1, &com));
  ros::Subscriber steeringLevelSubscriber = nh.subscribe<std_msgs::Int16>(
      "/uc_bridge/set_steering_level_msg", 1,
      boost::bind(uc_bridge::steeringLevelCallback, _1, &com));
  // raw mode subscriber
  ros::Subscriber ucBoardMsgSubscriber;
  if (rawComOn)
    ucBoardMsgSubscriber = nh.subscribe<std_msgs::String>(
        "/uc_bridge/send_uc_board_msg", 10,
        boost::bind(uc_bridge::ucBoardMessageCallback, _1, &com));

  // register shut down signal handler
  signal(SIGINT, uc_bridge::shutdownSignalHandler);

  ros::spin();
  return 0;
}
