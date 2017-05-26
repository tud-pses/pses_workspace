#include "pses_basis/serialinterface.h"

SerialInterface::SerialInterface()
{
  baudRate = 921600;
  deviceTag = "usb-FTDI_FT232R_USB_UART";
}

void SerialInterface::configure(unsigned int baudRate, std::string deviceTag)
{
  this->baudRate = baudRate;
  this->deviceTag = deviceTag;
}

void SerialInterface::connect(const unsigned int serialTimeout)
{
  if (serialConnection.isOpen())
    return;
  std::string deviceName;
  findDeviceName(deviceName);
  serialConnection.setPort(deviceName);
  serialConnection.setBaudrate(baudRate);
  serial::Timeout timeout = serial::Timeout::simpleTimeout(serialTimeout);
  serialConnection.setTimeout(timeout);
  serialConnection.open();
}

void SerialInterface::send(std::string& message)
{
  if (!serialConnection.isOpen())
    throw std::runtime_error("UC-board connection not established/lost.");
  serialConnection.write(message);
}

// be careful when using this method, it will block the calling thread until
// something has been
// received oder the request took longer than a certain timeout threshold
void SerialInterface::read(std::string& message, std::string& delimiter,
                           unsigned int maxLineLength)
{
  if (!serialConnection.isOpen())
    throw std::runtime_error("UC-board connection not established/lost.");
  if (serialConnection.waitReadable())
    serialConnection.readline(message, maxLineLength, delimiter);
}

void SerialInterface::disconnect()
{
  if (!serialConnection.isOpen())
    return;
  serialConnection.close();
}

void SerialInterface::findDeviceName(std::string& deviceName)
{
  std::string serialDevices = "/dev/serial/by-id/";
  std::string devicePath;

  struct dirent** fileList;
  int numOfFiles = scandir(serialDevices.c_str(), &fileList, NULL, alphasort);
  if (numOfFiles < 3)
  {
    throw std::runtime_error("No serial devices found.");
  }

  for (int i = 0; i < numOfFiles; i++)
  {

    if (std::string(fileList[i]->d_name).find(deviceTag) != std::string::npos)
    {
      devicePath.append(serialDevices);
      devicePath.append(std::string(fileList[i]->d_name));
      break;
    }
  }

  if (devicePath.length() <= 0)
  {
    throw std::runtime_error("UC-board is not connected.");
  }

  char buffer1[PATH_MAX + 1];
  size_t len = readlink(devicePath.c_str(), buffer1, sizeof(buffer1) - 1);
  if (len != -1)
  {
    buffer1[len] = '\0';
  }
  char buffer2[PATH_MAX + 1];
  realpath(serialDevices.append(std::string(buffer1)).c_str(), buffer2);
  deviceName = std::string(buffer2);
}
