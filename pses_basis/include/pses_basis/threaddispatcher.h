#ifndef THREADDISPATCHER_H
#define THREADDISPATCHER_H

#include <thread>
#include <string>
#include <pses_basis/serialinterface.h>
#include <ros/ros.h>

class ThreadDispatcher
{
public:
  ThreadDispatcher();
  void startReading(std::string delimiter);
  void stopReading();

private:
  bool doRead;
  std::thread serialReader;
  std::string delimiter;
  void readFromSerial();
};

#endif // THREADDISPATCHER_H
