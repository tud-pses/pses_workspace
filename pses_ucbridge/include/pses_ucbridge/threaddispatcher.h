#ifndef THREADDISPATCHER_H
#define THREADDISPATCHER_H

#include <string>
#include <queue>
#include <pses_ucbridge/serialinterface.h>
#include <pses_ucbridge/communicationthread.h>
#include <pses_ucbridge/readingthread.h>
#include <pses_ucbridge/communicationconfig.h>
#include <pses_ucbridge/sensorgroupthread.h>

class ReadingThread;
class SensorGroupThread;

typedef boost::function<void(const std::string&)> debugCallbackPtr;

class ThreadDispatcher : public CommunicationThread
{
public:
  ThreadDispatcher(const std::shared_ptr<Syntax>& syntax);
  void startThread();
  void stopThread();
  void enableDebugMessages(debugCallbackPtr debug);
  void enableRawCommunication();
  void setReadingThread(ReadingThread* rxThread);
  void setSensorGroupThread(SensorGroupThread* grpThread);
  void setCommunicationCondVar(std::condition_variable* condVar);
  void registerErrorCallback(debugCallbackPtr error);
  void registerTextCallback(debugCallbackPtr text);
  void dequeueResponse(std::string& response);
  void dequeueSensorGroupMessage(std::string& response);
  const bool IsResponseQueueEmpty() const;
  const bool IsMessageQueueEmpty() const;
  void setCommunicationWakeUp(bool wakeUp);

private:

  ReadingThread* readingThread;
  SensorGroupThread* sensorGroupThread;
  debugCallbackPtr debug;
  debugCallbackPtr error;
  debugCallbackPtr text;
  bool debugMsgEnabled;
  bool rawCommunicationEnabled;
  bool errorCBregistered;
  bool textCBregistered;
  std::shared_ptr<Syntax> syntax;
  std::queue<std::string> commandResponse;
  std::condition_variable* comCV;
  std::queue<std::string> sensorGroupMessage;

  void workerFunction();
  bool wakeUpCommunication;
};

#endif // THREADDISPATCHER_H
