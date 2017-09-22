#include <pses_basis/readingthread.h>

ReadingThread::ReadingThread(std::string delimiter,
                             ThreadDispatcher* dispatcher)
    : delimiter(delimiter), dispatcher(dispatcher)
{
  data = std::queue<std::string>();
}

void ReadingThread::startThread()
{
  //ROS_INFO_STREAM("ReadingThread starting..");
  active = true;
  worker = std::thread(&ReadingThread::workerFunction, this);
  //ROS_INFO_STREAM("ReadingThread started..");
}

void ReadingThread::stopThread()
{
  //ROS_INFO_STREAM("ReadingThread stopping..");
  active = false;
  wakeUp();
  worker.join();
  //ROS_INFO_STREAM("ReadingThread stopped..");
}

void ReadingThread::workerFunction()
{
  SerialInterface& si = SerialInterface::instance();
  while (active)
  {
    std::string message;
    try
    {
      si.read(message, delimiter);
      if (message.size() > 1)
      {
        //ROS_INFO_STREAM("ReadingThread pushing msg and waking ThreadDispatcher..");
        data.push(message);
        dispatcher->wakeUp();
      }
    }
    catch (std::exception& e)
    {
      ROS_ERROR("%s", e.what());
    }
  }
}

std::string ReadingThread::getData()
{
  if (data.size() > 0)
  {
    //ROS_INFO_STREAM("ThreadDispatcher getting Data..");
    std::string out(data.front());
    data.pop();
    return out;
  }
  else
  {
    return std::string("");
  }
}

const bool ReadingThread::isQueueEmpty() const{
  return data.empty();
}
