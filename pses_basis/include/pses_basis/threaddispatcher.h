#ifndef THREADDISPATCHER_H
#define THREADDISPATCHER_H

#include <string>
#include <pses_basis/serialinterface.h>
#include <pses_basis/communicationthread.h>
#include <pses_basis/readingthread.h>
#include <ros/ros.h>

class ReadingThread;

class ThreadDispatcher : public CommunicationThread
{
public:
  ThreadDispatcher(std::string delimiter);
  ~ThreadDispatcher();

  void startThread();
  void stopThread();

private:
  void workerFunction();
  ReadingThread* readingThread;
};

#endif // THREADDISPATCHER_H
