#include <ros/ros.h>
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
#include <pses_basis/ToggleUS.h>
#include <std_msgs/String.h>
#include <pses_basis/base64Decoder.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  std::string test = "BNz/EQHI/w";
  long mx = base64_decode(test, 1, 2, 2, true);
  long my = base64_decode(test, 3, 4, 2, true);
  long mz = base64_decode(test, 5, 6, 2, true);
  ROS_INFO_STREAM("mx: "<<mx<<" my: "<<my<<" mz: "<<mz);

  /*
  ros::Publisher pub = nh.advertise<std_msgs::String>("send_uc_board_msg", 10);
  std_msgs::String st;
  ros::Duration(1.0).sleep();
  st.data = "?ID\n";
  pub.publish(st);
  ros::Duration(0.5).sleep();
  st.data = "?ID\n";
  pub.publish(st);
  ros::Duration(0.5).sleep();
  st.data = "?STEER\n";
  pub.publish(st);
  ros::Duration(0.5).sleep();
  st.data = "?DRV\n";
  pub.publish(st);
  ros::spin();
  */

  /*
  ros::Duration(1.0).sleep();
  //test set_steering_level
  pses_basis::SetSteering::Request req13;
  req13.steering = 500;
  pses_basis::SetSteering::Response res13;
  ROS_INFO_STREAM("Sending Set Steering Level 500..");
  ros::Time t = ros::Time::now();
  ros::service::call("set_steering_level", req13, res13);
  ROS_INFO_STREAM("Round trip t: "<<(ros::Time::now()-t).toSec());
  int result13 = res13.was_set;
  ROS_INFO_STREAM("Received Set Steering Level, Answer: "<<result13);
  ros::Duration(0.01).sleep();
  req13.steering = 0;
  t = ros::Time::now();
  ros::service::call("set_steering_level", req13, res13);
  ROS_INFO_STREAM("Round trip t: "<<(ros::Time::now()-t).toSec());
  result13 = res13.was_set;
  ROS_INFO_STREAM("Received Set Steering Level, Answer: "<<result13);
  ros::Duration(0.01).sleep();
  req13.steering = -500;
  t = ros::Time::now();
  ros::service::call("set_steering_level", req13, res13);
  ROS_INFO_STREAM("Round trip t: "<<(ros::Time::now()-t).toSec());
  result13 = res13.was_set;
  ROS_INFO_STREAM("Received Set Steering Level, Answer: "<<result13);
  ros::Duration(0.01).sleep();
  req13.steering = 0;
  t = ros::Time::now();
  ros::service::call("set_steering_level", req13, res13);
  ROS_INFO_STREAM("Round trip t: "<<(ros::Time::now()-t).toSec());
  result13 = res13.was_set;
  ROS_INFO_STREAM("Received Set Steering Level, Answer: "<<result13);
  ros::Duration(0.01).sleep();
  req13.steering = 500;
  t = ros::Time::now();
  ros::service::call("set_steering_level", req13, res13);
  ROS_INFO_STREAM("Round trip t: "<<(ros::Time::now()-t).toSec());
  result13 = res13.was_set;
  ROS_INFO_STREAM("Received Set Steering Level, Answer: "<<result13);
  ros::Duration(0.01).sleep();
  req13.steering = 0;
  t = ros::Time::now();
  ros::service::call("set_steering_level", req13, res13);
  ROS_INFO_STREAM("Round trip t: "<<(ros::Time::now()-t).toSec());
  result13 = res13.was_set;
  ROS_INFO_STREAM("Received Set Steering Level, Answer: "<<result13);
  ros::Duration(0.01).sleep();
  req13.steering = -500;
  t = ros::Time::now();
  ros::service::call("set_steering_level", req13, res13);
  ROS_INFO_STREAM("Round trip t: "<<(ros::Time::now()-t).toSec());
  result13 = res13.was_set;
  ROS_INFO_STREAM("Received Set Steering Level, Answer: "<<result13);
  ros::Duration(0.01).sleep();
  req13.steering = 0;
  t = ros::Time::now();
  ros::service::call("set_steering_level", req13, res13);
  ROS_INFO_STREAM("Round trip t: "<<(ros::Time::now()-t).toSec());
  result13 = res13.was_set;
  ROS_INFO_STREAM("Received Set Steering Level, Answer: "<<result13);
  ros::Duration(0.01).sleep();
  req13.steering = 500;
  t = ros::Time::now();
  ros::service::call("set_steering_level", req13, res13);
  ROS_INFO_STREAM("Round trip t: "<<(ros::Time::now()-t).toSec());
  result13 = res13.was_set;
  ROS_INFO_STREAM("Received Set Steering Level, Answer: "<<result13);
  ros::Duration(0.01).sleep();
  req13.steering = 0;
  t = ros::Time::now();
  ros::service::call("set_steering_level", req13, res13);
  ROS_INFO_STREAM("Round trip t: "<<(ros::Time::now()-t).toSec());
  result13 = res13.was_set;
  ROS_INFO_STREAM("Received Set Steering Level, Answer: "<<result13);
  ros::Duration(0.01).sleep();
  req13.steering = -500;
  t = ros::Time::now();
  ros::service::call("set_steering_level", req13, res13);
  ROS_INFO_STREAM("Round trip t: "<<(ros::Time::now()-t).toSec());
  result13 = res13.was_set;
  ROS_INFO_STREAM("Received Set Steering Level, Answer: "<<result13);
  ros::Duration(0.01).sleep();
  req13.steering = 0;
  t = ros::Time::now();
  ros::service::call("set_steering_level", req13, res13);
  ROS_INFO_STREAM("Round trip t: "<<(ros::Time::now()-t).toSec());
  result13 = res13.was_set;
  ROS_INFO_STREAM("Received Set Steering Level, Answer: "<<result13);
  ros::Duration(0.01).sleep();
  req13.steering = 500;
  t = ros::Time::now();
  ros::service::call("set_steering_level", req13, res13);
  ROS_INFO_STREAM("Round trip t: "<<(ros::Time::now()-t).toSec());
  result13 = res13.was_set;
  ROS_INFO_STREAM("Received Set Steering Level, Answer: "<<result13);
  ros::Duration(0.01).sleep();
  req13.steering = 0;
  t = ros::Time::now();
  ros::service::call("set_steering_level", req13, res13);
  ROS_INFO_STREAM("Round trip t: "<<(ros::Time::now()-t).toSec());
  result13 = res13.was_set;
  ROS_INFO_STREAM("Received Set Steering Level, Answer: "<<result13);
  ros::Duration(0.01).sleep();
  req13.steering = -500;
  t = ros::Time::now();
  ros::service::call("set_steering_level", req13, res13);
  ROS_INFO_STREAM("Round trip t: "<<(ros::Time::now()-t).toSec());
  result13 = res13.was_set;
  ROS_INFO_STREAM("Received Set Steering Level, Answer: "<<result13);
  ros::Duration(0.01).sleep();
  req13.steering = 0;
  t = ros::Time::now();
  ros::service::call("set_steering_level", req13, res13);
  ROS_INFO_STREAM("Round trip t: "<<(ros::Time::now()-t).toSec());
  result13 = res13.was_set;
  ROS_INFO_STREAM("Received Set Steering Level, Answer: "<<result13);
  ros::Duration(0.01).sleep();
  req13.steering = 500;
  t = ros::Time::now();
  ros::service::call("set_steering_level", req13, res13);
  ROS_INFO_STREAM("Round trip t: "<<(ros::Time::now()-t).toSec());
  result13 = res13.was_set;
  ROS_INFO_STREAM("Received Set Steering Level, Answer: "<<result13);
  ros::Duration(0.01).sleep();

  /*
  //test delete_group
  pses_basis::DeleteGroup::Request req1;
  req1.group_number = 1;
  pses_basis::DeleteGroup::Response res1;
  ROS_INFO_STREAM("Sending Delete Group 1 ..");
  ros::service::call("delete_group", req1, res1);
  int result1 = res1.was_set;
  ROS_INFO_STREAM("Received Delete Group, Answer: "<<result1);
  ros::Duration(1.0).sleep();
  */

  /*
  //test get_controller_id
  pses_basis::GetControllerID::Request req2;
  pses_basis::GetControllerID::Response res2;
  ROS_INFO_STREAM("Sending Get Controller ID..");
  ros::service::call("get_controller_id", req2, res2);
  int result2 = res2.answer_received;
  int id1 = res2.ID;
  ROS_INFO_STREAM("Received Get Controller ID, Answer: "<<result2<<" ID: "<<id1);
  ros::Duration(1.0).sleep();


  //test get_daq_status
  pses_basis::GetDAQStatus::Request req3;
  pses_basis::GetDAQStatus::Response res3;
  ROS_INFO_STREAM("Sending Get DAQ status..");
  ros::service::call("get_daq_status", req3, res3);
  int result3 = res3.answer_received;
  std::string info1 = res3.status;
  ROS_INFO_STREAM("Received Get DAQ status, Answer: "<<result3<< " Status: "<<info1);
  ros::Duration(1.0).sleep();


  //test get_firmware_version
  pses_basis::GetFirmwareVersion::Request req4;
  pses_basis::GetFirmwareVersion::Response res4;
  ROS_INFO_STREAM("Sending Get Firmware Version..");
  ros::service::call("get_firmware_version", req4, res4);
  int result4 = res4.answer_received;
  std::string vers1 = res4.version;
  ROS_INFO_STREAM("Received Get Firmware Version, Answer: "<<result4<<" Version: "<<vers1);
  ros::Duration(1.0).sleep();

  //test get_info_all_groups
  pses_basis::GetInfoAllGroups::Request req5;
  pses_basis::GetInfoAllGroups::Response res5;
  ROS_INFO_STREAM("Sending Get Info all Groups..");
  ros::service::call("get_info_all_groups", req5, res5);
  int result5 = res5.answer_received;
  std::string info2 = res5.info;
  ROS_INFO_STREAM("Received Get Info all Groups, Answer: "<<result5<<" Info: "<<info2);
  ros::Duration(1.0).sleep();

  //test get_info_group
  pses_basis::GetInfoGroup::Request req6;
  req6.group_number = 2;
  pses_basis::GetInfoGroup::Response res6;
  ROS_INFO_STREAM("Sending Get Info Group 6..");
  ros::service::call("get_info_group", req6, res6);
  int result6 = res6.answer_received;
  std::string info3 = res6.info;
  ROS_INFO_STREAM("Received Get Info Group 6, Answer: "<<result6<<" Info: "<<info3);
  ros::Duration(1.0).sleep();


  //test get_kinect_status
  pses_basis::GetKinectStatus::Request req7;
  pses_basis::GetKinectStatus::Response res7;
  ROS_INFO_STREAM("Sending Get Kinect Status..");
  ros::service::call("get_kinect_status", req7, res7);
  int result7 = res7.answer_received;
  std::string status1 = res7.status;
  ROS_INFO_STREAM("Received Get Kinect Status, Answer: "<<result7<<" Status:"<<status1);
  ros::Duration(1.0).sleep();

  //test get_motor_level
  pses_basis::GetMotorLevel::Request req8;
  pses_basis::GetMotorLevel::Response res8;
  ROS_INFO_STREAM("Sending Get Motor level..");
  ros::service::call("get_motor_level", req8, res8);
  int result8 = res8.answer_received;
  int level1 = res8.level;
  ROS_INFO_STREAM("Received Get Motor level, Answer: "<<result8<<" level: "<<level1);
  ros::Duration(1.0).sleep();

  //test get_session_id
  pses_basis::GetSessionID::Request req9;
  pses_basis::GetSessionID::Response res9;
  ROS_INFO_STREAM("Sending Get Session ID..");
  ros::service::call("get_session_id", req9, res9);
  int result9 = res9.answer_received;
  int id2 = res9.SID;
  ROS_INFO_STREAM("Received Get Session ID, Answer: "<<result9<<" sid: "<<id2);
  ros::Duration(1.0).sleep();

  //test get_steering_level
  pses_basis::GetSteeringLevel::Request req10;
  pses_basis::GetSteeringLevel::Response res10;
  ROS_INFO_STREAM("Sending Get Steering Level..");
  ros::service::call("get_steering_level", req10, res10);
  int result10 = res10.answer_received;
  short level2 = res10.level;
  ROS_INFO_STREAM("Received Get Steering Level, Answer: "<<result10<<" level: "<<level2);
  ros::Duration(1.0).sleep();

  //test reset_controller
  pses_basis::ResetController::Request req11;
  pses_basis::ResetController::Response res11;
  ROS_INFO_STREAM("Sending Reset Controller..");
  ros::service::call("reset_controller", req11, res11);
  int result11 = res11.was_set;
  ROS_INFO_STREAM("Received Reset Controller, Answer: "<<result11);
  ros::Duration(1.0).sleep();


  //test set_motor_level
  pses_basis::SetMotorLevel::Request req12;
  req12.level = 5;
  pses_basis::SetMotorLevel::Response res12;
  ROS_INFO_STREAM("Sending Set Motor Level 5..");
  ros::service::call("set_motor_level", req12, res12);
  int result12 = res12.was_set;
  ROS_INFO_STREAM("Received Set Motor Level, Answer: "<<result12);
  ros::Duration(1.0).sleep();

  //test set_session_id
  pses_basis::SetSessionID::Request req131;
  req131.sid = 2;
  pses_basis::SetSessionID::Response res131;
  ROS_INFO_STREAM("Sending Set Session ID 2..");
  ros::service::call("set_session_id", req131, res131);
  int result131 = res131.was_set;
  ROS_INFO_STREAM("Received Set Session ID, Answer: "<<result131);
  ros::Duration(1.0).sleep();

  //test set_steering_level
  pses_basis::SetSteering::Request req13;
  req13.steering = 5;
  pses_basis::SetSteering::Response res13;
  ROS_INFO_STREAM("Sending Set Steering Level 5..");
  ros::service::call("set_steering_level", req13, res13);
  int result13 = res13.was_set;
  ROS_INFO_STREAM("Received Set Steering Level, Answer: "<<result13);
  ros::Duration(1.0).sleep();

  //test toggle_brakes
  pses_basis::ToggleBrakes::Request req14;
  req14.brakes_on = true;
  pses_basis::ToggleBrakes::Response res14;
  ROS_INFO_STREAM("Sending Toggle Brakes true..");
  ros::service::call("toggle_brakes", req14, res14);
  int result14 = res14.was_set;
  ROS_INFO_STREAM("Received toggle brakes, Answer: "<<result14);
  ros::Duration(1.0).sleep();
  */
  /*
  //test toggle_group
  pses_basis::ToggleGroup::Request req16;
  req16.group_on = false;
  req16.group_number = 1;
  pses_basis::ToggleGroup::Response res16;
  ROS_INFO_STREAM("Sending Toggle Group 1 false..");
  ros::service::call("toggle_group", req16, res16);
  int result16 = res16.was_set;
  ROS_INFO_STREAM("Received Toggle Group, Answer: "<<result16);
  ros::Duration(1.0).sleep();

  req16.group_number = 3;
  ROS_INFO_STREAM("Sending Toggle Group 1 false..");
  ros::service::call("toggle_group", req16, res16);
  result16 = res16.was_set;
  ROS_INFO_STREAM("Received Toggle Group, Answer: "<<result16);
  ros::Duration(1.0).sleep();

  req16.group_number = 4;
  ROS_INFO_STREAM("Sending Toggle Group 1 false..");
  ros::service::call("toggle_group", req16, res16);
  result16 = res16.was_set;
  ROS_INFO_STREAM("Received Toggle Group, Answer: "<<result16);
  ros::Duration(1.0).sleep();

  req16.group_number = 5;
  ROS_INFO_STREAM("Sending Toggle Group 1 false..");
  ros::service::call("toggle_group", req16, res16);
  result16 = res16.was_set;
  ROS_INFO_STREAM("Received Toggle Group, Answer: "<<result16);
  ros::Duration(1.0).sleep();

  //test toggle_daq
  pses_basis::ToggleDAQ::Request req15;
  req15.DAQ_on = true;
  pses_basis::ToggleDAQ::Response res15;
  ROS_INFO_STREAM("Sending Toggle DAQ true..");
  ros::service::call("toggle_daq", req15, res15);
  int result15 = res15.was_set;
  ROS_INFO_STREAM("Received toggle daq, Answer: "<<result15);
  ros::Duration(10).sleep();

  req15.DAQ_on = false;
  ROS_INFO_STREAM("Sending Toggle DAQ true..");
  ros::service::call("toggle_daq", req15, res15);
  result15 = res15.was_set;
  ROS_INFO_STREAM("Received toggle daq, Answer: "<<result15);
  ros::Duration(1).sleep();

  //test toggle_kinect
  pses_basis::ToggleKinect::Request req17;
  req17.kinect_on = false;
  pses_basis::ToggleKinect::Response res17;
  ROS_INFO_STREAM("Sending Toggle Kinect true..");
  ros::service::call("toggle_kinect", req17, res17);
  int result17 = res17.was_set;
  ROS_INFO_STREAM("Received Toggle Kinect, Answer: "<<result17);
  ros::Duration(1.0).sleep();

  //test toggle_motor
  pses_basis::ToggleMotor::Request req18;
  req18.motor_on = true;
  pses_basis::ToggleMotor::Response res18;
  ROS_INFO_STREAM("Sending Toggle Motor true..");
  ros::service::call("toggle_motor", req18, res18);
  int result18 = res18.was_set;
  ROS_INFO_STREAM("Received Toggle Motor, Answer: "<<result18);
  ros::Duration(1.0).sleep();

  //test toggle_us
  pses_basis::ToggleUS::Request req19;
  req19.us_on = true;
  pses_basis::ToggleUS::Response res19;
  ROS_INFO_STREAM("Sending Toggle us true..");
  ros::service::call("toggle_us", req19, res19);
  int result19 = res19.was_set;
  ROS_INFO_STREAM("Received Toggle Motor, Answer: "<<result19);
  ros::Duration(1.0).sleep();
  */

}
