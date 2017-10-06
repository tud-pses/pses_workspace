#include <ros/ros.h>
#include <pses_basis/SetMotorLevel.h>
#include <pses_basis/communication.h>
#include <pses_basis/communicationconfig.h>
#include <ros/package.h>
#include <sensor_msgs/Range.h>

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

void sendSensorGroupMessage1(
    SensorGroup* grp, std::unordered_map<std::string, ros::Publisher*>& pub)
{
  // do stuff
  //ROS_INFO_STREAM("doing stuff");
  sensor_msgs::Range us;
  unsigned short value;
  grp->getChannelValue("USL",  value);
  us.range = value;
  //ROS_INFO_STREAM(us);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uc_bridge");
  ros::NodeHandle nh;
  std::string typesPath = ros::package::getPath("pses_basis") + "/data/";
  Communication com(typesPath);
  ros::Publisher grp1 = nh.advertise<sensor_msgs::Range>("USL", 10);
  std::unordered_map<std::string, ros::Publisher*> usGrp;
  usGrp.insert(std::make_pair("USL", &grp1));
  com.registerSensorGroupCallback(1, boost::bind(&sendSensorGroupMessage1, _1, usGrp));
  com.registerSensorGroups("Set Group");

  ros::ServiceServer service =
      nh.advertiseService<pses_basis::SetMotorLevel::Request,
                          pses_basis::SetMotorLevel::Response>(
          "set_motor_level", std::bind(setMotorLevel, std::placeholders::_1,
                                       std::placeholders::_2, &com));

  com.sensorGroups[1].processResponse("3829");

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
