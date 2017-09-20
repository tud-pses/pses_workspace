#ifndef READINGTHREAD_H
#define READINGTHREAD_H

#include <string>
#include <queue>
#include <pses_basis/communicationthread.h>
#include <pses_basis/threaddispatcher.h>

class ThreadDispatcher;

class ReadingThread : public CommunicationThread
{
public:
  ReadingThread(std::string delimiter, ThreadDispatcher* dispatcher);
  void startThread();
  void stopThread();
  std::string getData();
  const bool isQueueEmpty() const;
private:
  std::string delimiter;
  std::queue<std::string> data;
  ThreadDispatcher* dispatcher;
  void workerFunction();
};

#endif // READINGTHREAD_H
