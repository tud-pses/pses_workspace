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
  void configure(unsigned int baudRate, std::string deviceTag);
  void connect(const unsigned int serialTimeout = 1000);
  void send(std::string& message);
  void read(std::string& message, std::string& delimiter,
            unsigned int maxLineLength = 65536);
  void disconnect();

private:
  // private class attributes
  unsigned int baudRate;
  std::string deviceTag;
  serial::Serial serialConnection;
  // private methods & constructors
  SerialInterface();
  SerialInterface(const SerialInterface&);
  SerialInterface& operator=(const SerialInterface&);

  void findDeviceName(std::string& deviceName);
};

#endif // SERIALINTERFACE_H
