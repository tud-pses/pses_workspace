#ifndef READINGTHREAD_H
#define READINGTHREAD_H

#include <string>
#include <queue>
#include <pses_ucbridge/communicationconfig.h>
#include <pses_ucbridge/communicationthread.h>
#include <pses_ucbridge/threaddispatcher.h>
#include <boost/range/algorithm/remove_if.hpp>

class ThreadDispatcher;

typedef boost::function<void(const std::string&)> errorCallbackPtr;

class ReadingThread : public CommunicationThread
{
public:
  ReadingThread(std::shared_ptr<Syntax> syntax, ThreadDispatcher* dispatcher);
  void startThread();
  void stopThread();
  void registerErrorCallback(errorCallbackPtr error);
  std::string getData();
  const bool isQueueEmpty() const;
private:
  std::shared_ptr<Syntax> syntax;
  std::queue<std::string> data;
  ThreadDispatcher* dispatcher;
  errorCallbackPtr error;
  bool errorCBregistered;
  void workerFunction();
};

#endif // READINGTHREAD_H
