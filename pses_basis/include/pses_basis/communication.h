#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <string>
#include <pses_basis/threaddispatcher.h>
#include <pses_basis/serialinterface.h>
#include <pses_basis/communicationconfig.h>
#include <pses_basis/readingthread.h>
#include <ros/ros.h>
#include <pses_basis/parameter.h>
#include <pses_basis/command.h>

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
                   Parameter::ParameterMap& outputParams,
                   unsigned int timeout = 10000);
  bool sendCommand(const std::string& command,
                   const std::vector<std::string>& options,
                   const Parameter::ParameterMap& inputParams,
                   Parameter::ParameterMap& outputParams,
                   unsigned int timeout = 10000);
  bool registerSensorGroups(const std::string& cmdName,
                            unsigned int timeout = 10000);
  void registerSensorGroupCallback(const unsigned char& grpNumber,
                                   responseCallback cbPtr);
private:
  CommunicationConfig comCfg;
  std::shared_ptr<Syntax> syntax;
  std::unordered_map<std::string, std::shared_ptr<Command> > commands;
  std::unordered_map<unsigned char, std::shared_ptr<SensorGroup> > sensorGroups;
  ThreadDispatcher* dispatcher;
  ReadingThread* rxPolling;
  SensorGroupThread* sensorGroupThread;
  mutable std::mutex mtx;
  std::condition_variable cv;
};

#endif // COMMUNICATION_H
