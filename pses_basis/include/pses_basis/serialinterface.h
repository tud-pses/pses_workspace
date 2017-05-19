#ifndef SERIALINTERFACE_H
#define SERIALINTERFACE_H

#include <serial/serial.h>
#include <string>
#include <dirent.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>

class SerialInterface
{
public:
  static SerialInterface& instance()
  {
    static SerialInterface _instance;
    return _instance;
  }
  ~SerialInterface() {}
  void configure(unsigned int baudRate, std::string deviceTag);
  int connect(const unsigned int serialTimeout = 5);

private:
  // private class attributes
  unsigned int baudRate;
  std::string deviceTag;
  serial::Serial serialConnection;
  bool connected;
  // private methods & constructors
  SerialInterface();
  SerialInterface(const SerialInterface&);
  SerialInterface& operator=(const SerialInterface&);

  int findDeviceName(std::string& deviceName);
};

#endif // SERIALINTERFACE_H
