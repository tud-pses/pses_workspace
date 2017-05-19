#include "pses_basis/serialinterface.h"

SerialInterface::SerialInterface()
{
  connected = false;
  baudRate = 921600;
  deviceTag = "usb-FTDI_FT232R_USB_UART";
}

void SerialInterface::configure(unsigned int baudRate, std::string deviceTag)
{
  this->baudRate = baudRate;
  this->deviceTag = deviceTag;
}

int SerialInterface::connect(const unsigned int serialTimeout)
{
  std::string deviceName;
  if (findDeviceName(deviceName) < 0)
  {
    return -1;
  }
  serialConnection.setPort(deviceName);
  serialConnection.setBaudrate(baudRate);
  serial::Timeout timeout = serial::Timeout::simpleTimeout(serialTimeout);
  serialConnection.setTimeout(timeout);

  try
  {
    serialConnection.open();
    return 1;
  }
  catch (serial::IOException& e)
  {
    std::cout<<e.what()<<std::endl;
    return -2;
  }
}

int SerialInterface::findDeviceName(std::string& deviceName)
{
  std::string serialDevices = "/dev/serial/by-id/";
  std::string devicePath;

  struct dirent** fileList;
  int noOfFiles = scandir(serialDevices.c_str(), &fileList, NULL, alphasort);
  if (noOfFiles < 3)
  {
    return -1;
  }

  for (int i = 0; i < noOfFiles; i++)
  {
    if (strstr(fileList[i]->d_name, deviceTag.c_str()) != 0)
    {
      devicePath.append(serialDevices);
      devicePath.append(std::string(fileList[i]->d_name));
    }
  }
  if (devicePath.length() <= 0)
  {
    return -2;
  }

  char buffer1[PATH_MAX];
  readlink(devicePath.c_str(), buffer1, sizeof(buffer1) - 1);
  char buffer2[PATH_MAX];
  realpath(buffer1, buffer2);
  std::string serialPortName(buffer2);

  deviceName = "/dev";
  deviceName.append(serialPortName);
  stripIllegal(deviceName);

  return 1;
}

void SerialInterface::stripIllegal(std::string& str)
{
  str.erase(remove_if(str.begin(), str.end(), invalidChar), str.end());
}
