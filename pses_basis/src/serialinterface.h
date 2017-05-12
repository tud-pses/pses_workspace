#ifndef SERIALINTERFACE_H
#define SERIALINTERFACE_H

#include <serial/serial.h>
#include <string>

class SerialInterface
{
public:
  static SerialInterface& instance()
  {
    static SerialInterface _instance;
    return _instance;
  }
  ~SerialInterface() {}
  void configure(unsigned int baudRate, std::string deviceName);
  void connect(const unsigned int serialTimeout = 5);

private:
  // private class attributes
  unsigned int baudRate;
  std::string deviceName;
  serial::Serial serialConnection;
  bool connected;
  // priva methods & constructors
  SerialInterface();
  SerialInterface(const SerialInterface&);
  SerialInterface& operator=(const SerialInterface&);
};

#endif // SERIALINTERFACE_H
