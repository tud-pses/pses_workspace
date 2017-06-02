#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <string>
#include <pses_basis/threaddispatcher.h>
#include <pses_basis/serialinterface.h>
#include <ros/ros.h>


class Communication
{
public:
  Communication();
  ~Communication();
  void connect();
  void startCommunication();
  void stopCommunication();
  void disconnect();

private:
  ThreadDispatcher* dispatcher;


};

#endif // COMMUNICATION_H
