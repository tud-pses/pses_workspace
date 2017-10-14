#include <pses_ucbridge/serialinterface.h>

SerialInterface::SerialInterface()
{
  baudRate = 921600;
  deviceTag = "usb-FTDI_FT232R_USB_UART";
  serialTimeout = 1000;
  maxLineLength = 65536;
  serialDevicesFolder = "/dev/serial/by-id/";
}

void SerialInterface::setBaudRate(unsigned int baudRate){
  this->baudRate = baudRate;
}

void SerialInterface::setDeviceTag(const std::string& deviceTag){
  this->deviceTag = deviceTag;
}

void SerialInterface::setSerialTimeout(unsigned int serialTimeout){
  this->serialTimeout = serialTimeout;
}

void SerialInterface::setMaxLineLength(unsigned int maxLineLength){
  this->maxLineLength = maxLineLength;
}

void SerialInterface::setSerialDevicesFolder(const std::string& serialDevicesFolder){
  this->serialDevicesFolder = serialDevicesFolder;
}

void SerialInterface::connect()
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
// received or the request took longer than a certain timeout threshold
void SerialInterface::read(std::string& message, std::string& delimiter)
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
  std::string serialDevices = serialDevicesFolder;
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
