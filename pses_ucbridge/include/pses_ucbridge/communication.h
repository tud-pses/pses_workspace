#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <string>
#include <pses_ucbridge/threaddispatcher.h>
#include <pses_ucbridge/serialinterface.h>
#include <pses_ucbridge/communicationconfig.h>
#include <pses_ucbridge/readingthread.h>
#include <ros/ros.h>
#include <pses_ucbridge/parameter.h>
#include <pses_ucbridge/command.h>

class Communication
{
public:
  Communication(const std::string& configPath);
  ~Communication();
  void connect();
  void startCommunication();
  void stopCommunication();
  void disconnect();
  void sendRawMessage(const std::string& msg);
  void enableDebugMessages(debugCallbackPtr debug);
  void enableRawCommunication();
  bool sendCommand(const std::string& command,
                   const Parameter::ParameterMap& inputParams,
                   Parameter::ParameterMap& outputParams,
                   unsigned int timeout = 100000);
  bool sendCommand(const std::string& command,
                   const std::vector<std::string>& options,
                   const Parameter::ParameterMap& inputParams,
                   Parameter::ParameterMap& outputParams,
                   unsigned int timeout = 100000);
  void registerErrorCallback(debugCallbackPtr error);
  void registerTextCallback(debugCallbackPtr text);
  bool registerSensorGroups(const std::string& cmdName,
                            unsigned int timeout = 100000);
  void registerSensorGroupCallback(const unsigned char& grpNumber,
                                   responseCallback cbPtr);
private:
  CommunicationConfig comCfg;
  std::shared_ptr<SerialInterfaceParams> serialParams;
  std::shared_ptr<Syntax> syntax;
  std::unordered_map<std::string, std::shared_ptr<Command> > commands;
  std::unordered_map<unsigned char, std::shared_ptr<SensorGroup> > sensorGroups;
  ThreadDispatcher* dispatcher;
  bool rawCommunicationEnabled;
  ReadingThread* rxPolling;
  SensorGroupThread* sensorGroupThread;
  mutable std::mutex mtx;
  std::condition_variable cv;

  void configSerialInterface();
};

#endif // COMMUNICATION_H
