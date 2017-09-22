#ifndef THREADDISPATCHER_H
#define THREADDISPATCHER_H

#include <string>
#include <queue>
#include <pses_basis/serialinterface.h>
#include <pses_basis/communicationthread.h>
#include <pses_basis/readingthread.h>
#include <pses_basis/communicationconfig.h>
#include <ros/ros.h>

class ReadingThread;

class ThreadDispatcher : public CommunicationThread
{
public:
  ThreadDispatcher(const Syntax* syntax);
  void startThread();
  void stopThread();

  void setReadingThread(ReadingThread* rxThread);
  void setCommunicationCondVar(std::condition_variable* condVar);
  void dequeueResponse(std::string response);
  const bool IsResponseQueueEmpty() const;
  void setCommunicationWakeUp(bool wakeUp);

private:
  void workerFunction();
  ReadingThread* readingThread;
  const Syntax* syntax;
  std::queue<std::string> commandResponse;
  bool wakeUpCommunication;
  std::condition_variable* comCV;
};

#endif // THREADDISPATCHER_H
