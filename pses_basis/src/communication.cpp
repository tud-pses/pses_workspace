#include <pses_basis/communication.h>

Communication::Communication(const std::string& configPath)
{
  comCfg = CommunicationConfig(configPath);
  //comCfg.readDataTypes();
  comCfg.readGeneralSyntax();
  comCfg.readCommands();
  commands = comCfg.getCommands();
  dispatcher = new ThreadDispatcher(comCfg.getSyntax());
  rxPolling = new ReadingThread(comCfg.getSyntax()->endOfMessage, dispatcher);
  dispatcher->setReadingThread(rxPolling);
  dispatcher->setCommunicationCondVar(&cv);
}

Communication::~Communication()
{
  delete (dispatcher);
  delete (rxPolling);
}

void Communication::connect()
{
  SerialInterface& si = SerialInterface::instance();
  si.connect();
}

void Communication::startCommunication()
{
  SerialInterface& si = SerialInterface::instance();
  dispatcher->startThread();
  /*
  std::string msg = std::string("?ID\n");
  si.send(msg);
  ros::Duration(0.5).sleep();
  msg = "!DAQ GRP 1 ~TS=10 ~AVG AX AY AZ\n";
  si.send(msg);
  ros::Duration(0.5).sleep();
  msg = "!DAQ START\n";
  si.send(msg);
  ros::Duration(10.0).sleep();
  */
}

void Communication::stopCommunication()
{
  SerialInterface& si = SerialInterface::instance();
  /*
  std::string msg = "!DAQ STOP\n";
  si.send(msg);
  ros::Duration(0.5).sleep();
  */
  dispatcher->stopThread();
  si.disconnect();
}

void Communication::disconnect()
{
  SerialInterface& si = SerialInterface::instance();
  si.disconnect();
}
//timeout in microseconds
bool Communication::sendCommand(const std::string& command,
                                const Parameter::ParameterMap& inputParams,
                                Parameter::ParameterMap& outputParams,
                                unsigned int timeout)
{
  SerialInterface& si = SerialInterface::instance();
  std::unique_lock<std::mutex> lck(mtx);
  std::string cmd;
  std::string response = "F 6";
  commands[command].generateCommand(inputParams, cmd);
  ROS_INFO_STREAM(cmd);
  dispatcher->setCommunicationWakeUp(true);
  // si.send(cmd);
  cv.wait_for(lck, std::chrono::microseconds(timeout));
  //if(dispatcher->IsResponseQueueEmpty()) return false;
  //dispatcher->dequeueResponse(response);
  return commands[command].verifyResponse(inputParams, response, outputParams);
}
