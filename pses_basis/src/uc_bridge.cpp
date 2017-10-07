#include <ros/ros.h>
#include <pses_basis/servicefunctions.h>
#include <pses_basis/communication.h>
#include <pses_basis/communicationconfig.h>
#include <ros/package.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>



void publishSensorGroupMessage1(
    SensorGroup* grp, std::unordered_map<std::string, ros::Publisher*>& pub)
{
  // do stuff
  //ROS_INFO_STREAM("doing stuff");
  sensor_msgs::Range us;
  unsigned short value;
  grp->getChannelValue("USL",  value);
  us.range = value;
  ROS_INFO_STREAM(us);
  pub["USL"]->publish(us);
  grp->getChannelValue("USF",  value);
  us.range = value;
  ROS_INFO_STREAM(us);
  pub["USF"]->publish(us);
  grp->getChannelValue("USR",  value);
  us.range = value;
  ROS_INFO_STREAM(us);
  pub["USR"]->publish(us);
}

void publishSensorGroupMessage2(
    SensorGroup* grp, std::unordered_map<std::string, ros::Publisher*>& pub)
{
  // do stuff
  //ROS_INFO_STREAM("doing stuff");
  sensor_msgs::Imu imu;
  short gx;
  short gy;
  short gz;
  grp->getChannelValue("GX",  gx);
  grp->getChannelValue("GY",  gy);
  grp->getChannelValue("GZ",  gz);
  imu.angular_velocity.x=gx;
  imu.angular_velocity.y=gy;
  imu.angular_velocity.z=gz;
  ROS_INFO_STREAM(imu);
  pub["IMU"]->publish(imu);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uc_bridge");
  ros::NodeHandle nh;
  std::string typesPath = ros::package::getPath("pses_basis") + "/data/";
  Communication com(typesPath);
  ros::Publisher grp1 = nh.advertise<sensor_msgs::Range>("USL", 10);
  ros::Publisher grp12 = nh.advertise<sensor_msgs::Range>("USF", 10);
  ros::Publisher grp13 = nh.advertise<sensor_msgs::Range>("USR", 10);
  ros::Publisher grp2 = nh.advertise<sensor_msgs::Imu>("IMU", 10);
  std::unordered_map<std::string, ros::Publisher*> usGrp;
  std::unordered_map<std::string, ros::Publisher*> imuGrp;
  usGrp.insert(std::make_pair("USL", &grp1));
  usGrp.insert(std::make_pair("USF", &grp12));
  usGrp.insert(std::make_pair("USR", &grp13));
  imuGrp.insert(std::make_pair("IMU", &grp2));
  com.registerSensorGroupCallback(1, boost::bind(&publishSensorGroupMessage1, _1, usGrp));
  com.registerSensorGroupCallback(2, boost::bind(&publishSensorGroupMessage2, _1, imuGrp));
  com.registerSensorGroups("Set Group");

  ros::ServiceServer setMotorLevelService =
      nh.advertiseService<pses_basis::SetMotorLevel::Request,
                          pses_basis::SetMotorLevel::Response>(
          "set_motor_level", std::bind(setMotorLevel, std::placeholders::_1,
                                       std::placeholders::_2, &com));
  ros::ServiceServer setSteeringLevelService =
      nh.advertiseService<pses_basis::SetSteering::Request,
                          pses_basis::SetSteering::Response>(
          "set_steering_level", std::bind(setSteeringLevel, std::placeholders::_1,
                                       std::placeholders::_2, &com));

  com.startCommunication();
  ros::Duration(0.01).sleep();
  com.stopCommunication();

  //com.sensorGroups[1]->processResponse("3829");

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
