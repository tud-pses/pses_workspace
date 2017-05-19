#include "ros/ros.h"
#include "pses_basis/SetMotorLevel.h"
#include "pses_basis/serialinterface.h"
#include <dirent.h>
#include <sys/stat.h>

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
  ros::ServiceServer service = nh.advertiseService("set_motor_level", setMotorLevel);
  //ROS_INFO_STREAM("tut was ...");
  //SerialInterface& si = SerialInterface::instance();
  //si.connect();
  //DIR *pDir = opendir("/dev/serial/by-id");
  //if(d == NULL) //Couldn't open directory
  //  return 0;
  char* path = "/dev/serial/by-id/";
  char* device = "usb-FTDI_FT232R_USB_UART_AI04O1IA-if00-port0";
  char dev_p[200];
  struct dirent **fileList;
  int noOfFiles = scandir(path, &fileList, NULL, alphasort);
  ROS_INFO_STREAM("Num of files: "<<(noOfFiles));
  for(int i = 0; i<noOfFiles; i++){
    ROS_INFO_STREAM(fileList[i]->d_name);
    if(strstr(fileList[i]->d_name,device)!=0){
      ROS_INFO_STREAM("Device Found");
      strcpy(dev_p, path);
      strcat(dev_p, device);
    }
  }

  char buf[1024];
  readlink(dev_p, buf, sizeof(buf)-1);
  char blubb[1024];
  realpath(buf, blubb);
  ROS_INFO_STREAM("Path: "<<dev_p<<" // Device id: "<< blubb);

  struct stat attr;
  stat(dev_p, &attr);
  ROS_INFO_STREAM("Device id: "<< attr.st_gid);



  ros::spin();

  return 0;
}
