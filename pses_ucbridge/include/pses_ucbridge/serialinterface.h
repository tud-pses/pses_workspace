#ifndef SERIALINTERFACE_H
#define SERIALINTERFACE_H

#include <serial/serial.h>
#include <string>
#include <dirent.h>
#include <stdlib.h>
#include <unistd.h>

class SerialInterface
{
public:
  static SerialInterface& instance()
  {
    static SerialInterface _instance;
    return _instance;
  }
  ~SerialInterface() {}
  void setBaudRate(unsigned int baudRate);
  void setDeviceTag(const std::string& deviceTag);
  void setSerialTimeout(unsigned int serialTimeout);
  void setMaxLineLength(unsigned int maxLineLength);
  void setSerialDevicesFolder(const std::string& serialDevicesFolder);
  void connect();
  void send(std::string& message);
  void read(std::string& message, std::string& delimiter);
  void disconnect();

private:
  // private class attributes
  unsigned int baudRate;
  std::string deviceTag;
  unsigned int serialTimeout;
  unsigned int  maxLineLength;
  std::string serialDevicesFolder;
  serial::Serial serialConnection;
  // private methods & constructors
  SerialInterface();
  SerialInterface(const SerialInterface&);
  SerialInterface& operator=(const SerialInterface&);

  void findDeviceName(std::string& deviceName);
};

#endif // SERIALINTERFACE_H
