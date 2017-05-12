#include "serialinterface.h"

SerialInterface::SerialInterface()
{
  connected = false;
  baudRate = 921600;
  deviceName = "/dev/ttyUSB0";
}

void SerialInterface::configure(const unsigned int baudRate, const std::string deviceName)
{
  this->baudRate = baudRate;
  this->deviceName = deviceName;
}

void SerialInterface::connect(const unsigned int serialTimeout)
{
  if (!connected)
  {
    try
    {
      serialConnection.setPort("/dev/" + deviceName);
      serialConnection.setBaudrate(baudRate);
      serial::Timeout timeout = serial::Timeout::simpleTimeout(serialTimeout);
      serialConnection.setTimeout(timeout);
      serialConnection.open();
    }
    catch (serial::IOException& e)
    {
      // throw UcBoardException(Board::CONNECTING_FAILED);
    }
    if (serialConnection.isOpen())
    {
      connected = true;
    }
    else
    {
      // throw UcBoardException(Board::CONNECTION_NOT_ESTABLISHED);
    }
  }
  else
  {
    // throw UcBoardException(Board::CONNECTION_ALREADY_ESTABLISHED);
  }
}
