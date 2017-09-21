#include "pses_basis/threaddispatcher.h"

ThreadDispatcher::ThreadDispatcher(const Syntax* syntax)
{
  this->syntax = syntax;
  commandResponse = std::queue<std::string>();
}

void ThreadDispatcher::startThread()
{
  // ROS_INFO_STREAM("ThreadDispatcher starting..");
  active = true;
  worker = std::thread(&ThreadDispatcher::workerFunction, this);
  readingThread->startThread();
  // ROS_INFO_STREAM("ThreadDispatcher started..");
}

void ThreadDispatcher::stopThread()
{
  // ROS_INFO_STREAM("ThreadDispatcher stopping..");
  readingThread->stopThread();
  active = false;
  wakeUp();
  worker.join();
  // ROS_INFO_STREAM("ThreadDispatcher stopped..");
}

void ThreadDispatcher::workerFunction()
{
  while (active)
  {
    // ROS_INFO_STREAM("ThreadDispatcher sleeping..");
    sleep();
    // ROS_INFO_STREAM("ThreadDispatcher woke up..");
    while (!readingThread->isQueueEmpty() && active)
    {
      std::string data = readingThread->getData();
      // in case of empty string
      if(data.size() < 1) continue;
      // check for known prefixes
      if (data.find(syntax->cmdErrorPrefix) == 0)
      {
        // dispatch cmd-error thread
      }else if(data.find(syntax->genErrorPrefix) == 0)
      {
        // dispatch general-error thread
      }
      else if (data.find(syntax->channelGrpMsgPrefix) == 0)
      {
        // dispatch grp thread
      }
      else if (data.find(syntax->answerOnCmdPrefix) == 0)
      {
        commandResponse.push(data);
      }
    }
  }
}

void ThreadDispatcher::setReadingThread(ReadingThread* rxThread)
{
  if (!active)
  {
    this->readingThread = rxThread;
  }
}

void ThreadDispatcher::dequeueResponse(std::string response){
  response = commandResponse.front();
  commandResponse.pop();
}

const bool ThreadDispatcher::IsResponseQueueEmpty() const {
  return commandResponse.empty();
}
