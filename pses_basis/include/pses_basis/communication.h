#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <string>
#include <pses_basis/threaddispatcher.h>
#include <pses_basis/serialinterface.h>
#include <pses_basis/communicationconfig.h>
#include <ros/ros.h>


class Communication
{
public:
  Communication(const std::string& configPath);
  ~Communication();
  void connect();
  void startCommunication();
  void stopCommunication();
  void disconnect();

private:
  CommunicationConfig comCfg;
  ThreadDispatcher* dispatcher;


};

#endif // COMMUNICATION_H
