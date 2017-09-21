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
  void dequeueResponse(std::string response);
  const bool IsResponseQueueEmpty() const;

private:
  void workerFunction();
  ReadingThread* readingThread;
  const Syntax* syntax;
  std::queue<std::string> commandResponse;
};

#endif // THREADDISPATCHER_H
