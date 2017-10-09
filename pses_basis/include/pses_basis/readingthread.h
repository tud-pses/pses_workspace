#ifndef READINGTHREAD_H
#define READINGTHREAD_H

#include <string>
#include <queue>
#include <pses_basis/communicationconfig.h>
#include <pses_basis/communicationthread.h>
#include <pses_basis/threaddispatcher.h>
#include <boost/range/algorithm/remove_if.hpp>

class ThreadDispatcher;

class ReadingThread : public CommunicationThread
{
public:
  ReadingThread(std::shared_ptr<Syntax> syntax, ThreadDispatcher* dispatcher);
  void startThread();
  void stopThread();
  std::string getData();
  const bool isQueueEmpty() const;
private:
  std::shared_ptr<Syntax> syntax;
  std::queue<std::string> data;
  ThreadDispatcher* dispatcher;
  void workerFunction();
};

#endif // READINGTHREAD_H
