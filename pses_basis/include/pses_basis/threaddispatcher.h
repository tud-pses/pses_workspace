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
  ThreadDispatcher(const Params::Syntax* syntax);
  void startThread();
  void stopThread();

  void setReadingThread(ReadingThread* rxThread);

private:
  void workerFunction();
  ReadingThread* readingThread;
  const Params::Syntax* syntax;
};

#endif // THREADDISPATCHER_H
