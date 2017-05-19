#include "pses_basis/serialinterface.h"

SerialInterface::SerialInterface()
{
  connected = false;
  baudRate = 921600;
  deviceName = "/dev/ttyUSB0";
}

void SerialInterface::configure(const unsigned int baudRate,
                                const std::string deviceName)
{
  this->baudRate = baudRate;
  this->deviceName = deviceName;
}

void SerialInterface::connect(const unsigned int serialTimeout)
{
  ROS_INFO_STREAM("Config Phase..");
  serialConnection.setPort(deviceName);
  serialConnection.setBaudrate(baudRate);
  serial::Timeout timeout = serial::Timeout::simpleTimeout(serialTimeout);
  serialConnection.setTimeout(timeout);

  try
  {
    serialConnection.open();
    ROS_INFO_STREAM("Connection open..");
  }
  catch (serial::IOException& e)
  {
    ROS_INFO_STREAM(e.what());
  }
}
