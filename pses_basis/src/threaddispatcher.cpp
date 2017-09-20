#include "pses_basis/threaddispatcher.h"

ThreadDispatcher::ThreadDispatcher(const Params::Syntax* syntax)
{
  this->syntax = syntax;
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
      if (data.find(syntax->cmdErrorPrefix) == 0)
      {
        // dispatch error thread
      }
      else if (data.find(syntax->channelGrpMsgPrefix) == 0)
      {
        // dispatch grp thread
      }
      else if (data.find(syntax->answerOnCmdPrefix) == 0)
      {
        // dispatch response thread
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
