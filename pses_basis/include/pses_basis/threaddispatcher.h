#ifndef THREADDISPATCHER_H
#define THREADDISPATCHER_H

#include <string>
#include <queue>
#include <pses_basis/serialinterface.h>
#include <pses_basis/communicationthread.h>
#include <pses_basis/readingthread.h>
#include <pses_basis/communicationconfig.h>
#include <pses_basis/sensorgroupthread.h>
#include <ros/ros.h>

class ReadingThread;
class SensorGroupThread;

class ThreadDispatcher : public CommunicationThread
{
public:
  ThreadDispatcher(const std::shared_ptr<Syntax>& syntax);
  void startThread();
  void stopThread();

  void setReadingThread(ReadingThread* rxThread);
  void setSensorGroupThread(SensorGroupThread* grpThread);
  void setCommunicationCondVar(std::condition_variable* condVar);
  void dequeueResponse(std::string& response);
  void dequeueSensorGroupMessage(std::string& response);
  const bool IsResponseQueueEmpty() const;
  const bool IsMessageQueueEmpty() const;
  void setCommunicationWakeUp(bool wakeUp);

private:

  ReadingThread* readingThread;
  SensorGroupThread* sensorGroupThread;
  std::shared_ptr<Syntax> syntax;
  std::queue<std::string> commandResponse;
  std::condition_variable* comCV;
  std::queue<std::string> sensorGroupMessage;

  void workerFunction();
  bool wakeUpCommunication;
};

#endif // THREADDISPATCHER_H
