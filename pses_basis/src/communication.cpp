#include <pses_basis/communication.h>

Communication::Communication(const std::string& configPath)
{
  // load config files
  comCfg = CommunicationConfig(configPath);
  serialParams = comCfg.getSerialInterfaceParams();
  configSerialInterface();
  rawCommunicationEnabled = false;
  syntax = comCfg.getSyntax();
  commands = comCfg.getCommands();
  sensorGroups = comCfg.getSensorGroups();
  dispatcher = new ThreadDispatcher(syntax);
  rxPolling = new ReadingThread(syntax, dispatcher);
  sensorGroupThread = new SensorGroupThread(syntax, dispatcher, sensorGroups);
  dispatcher->setReadingThread(rxPolling);
  dispatcher->setSensorGroupThread(sensorGroupThread);
  dispatcher->setCommunicationCondVar(&cv);
}

Communication::~Communication()
{
  delete (dispatcher);
  delete (rxPolling);
  delete (sensorGroupThread);
}

void Communication::configSerialInterface()
{
  SerialInterface& si = SerialInterface::instance();
  si.setBaudRate(serialParams->baudRate);
  si.setDeviceTag(serialParams->deviceTag);
  si.setMaxLineLength(serialParams->maxLineLength);
  si.setSerialDevicesFolder(serialParams->serialDevicesFolder);
  si.setSerialTimeout(serialParams->serialTimeout);
}

void Communication::connect()
{
  SerialInterface& si = SerialInterface::instance();
  si.connect();
}

void Communication::startCommunication() { dispatcher->startThread(); }

void Communication::stopCommunication() { dispatcher->stopThread(); }

void Communication::disconnect()
{
  SerialInterface& si = SerialInterface::instance();
  si.disconnect();
}

void Communication::enableDebugMessages(debugCallbackPtr debug)
{
  dispatcher->enableDebugMessages(debug);
  rawCommunicationEnabled = true;
}

void Communication::enableRawCommunication()
{
  dispatcher->enableRawCommunication();
}

void Communication::sendRawMessage(const std::string& msg)
{
  if (!rawCommunicationEnabled)
    throw std::runtime_error("Raw communication mode not enabled!");
  SerialInterface& si = SerialInterface::instance();
  std::string send = msg;
  si.send(send);
}

// timeout in microseconds
bool Communication::sendCommand(const std::string& command,
                                const Parameter::ParameterMap& inputParams,
                                Parameter::ParameterMap& outputParams,
                                unsigned int timeout)
{
  ros::Time t = ros::Time::now();
  SerialInterface& si = SerialInterface::instance();
  std::unique_lock<std::mutex> lck(mtx);
  std::string cmd;
  std::string response;
  commands[command]->generateCommand(inputParams, cmd);
  // ROS_INFO_STREAM(cmd);
  dispatcher->setCommunicationWakeUp(true);
  cmd = cmd + syntax->endOfFrame;
  ROS_INFO_STREAM("Cmd gen time t: " << (ros::Time::now() - t).toSec());
  t = ros::Time::now();
  si.send(cmd);
  cv.wait_for(lck, std::chrono::microseconds(timeout));
  if (dispatcher->IsResponseQueueEmpty())
    return false;
  // ROS_INFO_STREAM("Round trip t: "<<(ros::Time::now()-t).toSec());
  ROS_INFO_STREAM("Wakeup time t: " << (ros::Time::now() - t).toSec());
  t = ros::Time::now();
  while (!dispatcher->IsResponseQueueEmpty())
  {
    dispatcher->dequeueResponse(response);
    // ROS_INFO_STREAM(response);
    if (commands[command]->verifyResponse(inputParams, response, outputParams))
    {
      ROS_INFO_STREAM("Search time t: " << (ros::Time::now() - t).toSec());
      return true;
    }
  }
}
// timeout in microseconds
bool Communication::sendCommand(const std::string& command,
                                const std::vector<std::string>& options,
                                const Parameter::ParameterMap& inputParams,
                                Parameter::ParameterMap& outputParams,
                                unsigned int timeout)
{
  SerialInterface& si = SerialInterface::instance();
  std::unique_lock<std::mutex> lck(mtx);
  std::string cmd;
  std::string response = "F 6";
  commands[command]->generateCommand(inputParams, options, cmd);
  // ROS_INFO_STREAM(cmd);
  dispatcher->setCommunicationWakeUp(true);
  cmd = cmd + syntax->endOfFrame;
  // si.send(cmd);
  cv.wait_for(lck, std::chrono::microseconds(timeout));
  // if(dispatcher->IsResponseQueueEmpty()) return false;
  // dispatcher->dequeueResponse(response);
  // return commands[command]->verifyResponse(inputParams, options,response,
  // outputParams);
  return true;
}

bool Communication::registerSensorGroups(const std::string& cmdName,
                                         unsigned int timeout)
{
  bool success = true;
  SerialInterface& si = SerialInterface::instance();
  for (auto grp : sensorGroups)
  {
    std::unique_lock<std::mutex> lck(mtx);
    std::string cmd;
    std::string response;
    // ROS_INFO_STREAM("Setting cmd for: " <<grp.second->getName());
    grp.second->createSensorGroupCommand(*commands[cmdName], cmd);
    // ROS_INFO_STREAM(cmd);
    dispatcher->setCommunicationWakeUp(true);
    cmd = cmd + syntax->endOfFrame;
    // ros::Time t = ros::Time::now();
    si.send(cmd);
    cv.wait_for(lck, std::chrono::microseconds(timeout));
    // ROS_INFO_STREAM("Round trip t: "<<(ros::Time::now()-t).toSec());
    if (dispatcher->IsResponseQueueEmpty())
    {
      // ROS_INFO_STREAM("Response queue empty");
      success = false;
      continue;
    }
    dispatcher->dequeueResponse(response);
    // ROS_INFO_STREAM(response);
    if (!grp.second->verifyResponseOnComand(*commands[cmdName], response))
      success = false;
  }
  return success;
}

void Communication::registerErrorCallback(debugCallbackPtr error)
{
  dispatcher->registerErrorCallback(error);
  for(auto grp : sensorGroups){
    grp.second->registerErrorCallback(error);
  }
}

void Communication::registerTextCallback(debugCallbackPtr text)
{
  dispatcher->registerTextCallback(text);
}

void Communication::registerSensorGroupCallback(const unsigned char& grpNumber,
                                                responseCallback cbPtr)
{
  sensorGroups[grpNumber]->setResponseCallback(cbPtr);
}
