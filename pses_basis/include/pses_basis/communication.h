#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <string>
#include <pses_basis/threaddispatcher.h>
#include <pses_basis/serialinterface.h>
#include <pses_basis/communicationconfig.h>
#include "pses_basis/readingthread.h"
#include <ros/ros.h>
#include <pses_basis/parameter.h>

class Communication
{
public:
  Communication(const std::string& configPath);
  ~Communication();
  void connect();
  void startCommunication();
  void stopCommunication();
  void disconnect();
  bool sendCommand(const std::string& command,
                   const Parameter::ParameterMap& inputParams,
                   Parameter::ParameterMap& outputParams);

private:
  CommunicationConfig comCfg;
  ThreadDispatcher* dispatcher;
  ReadingThread* rxPolling;
};

#endif // COMMUNICATION_H
