#include <pses_basis/readingthread.h>

ReadingThread::ReadingThread(std::shared_ptr<Syntax> syntax,
                             ThreadDispatcher* dispatcher)
    : syntax(syntax), dispatcher(dispatcher)
{
  data = std::queue<std::string>();
}

void ReadingThread::startThread()
{
  // ROS_INFO_STREAM("ReadingThread starting..");
  active = true;
  worker = std::thread(&ReadingThread::workerFunction, this);
  // ROS_INFO_STREAM("ReadingThread started..");
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
      si.read(message, syntax->endOfFrame);
      message.erase(
          boost::remove_if(message, boost::is_any_of(syntax->endOfFrame +
                                                     syntax->endOfMessage)),
          message.end());
      //ROS_INFO_STREAM(message);
      if (message.size() > 1)
      {
        //ROS_INFO_STREAM(message);
        // ROS_INFO_STREAM("ReadingThread pushing msg and waking
        // ThreadDispatcher.. msg: "<<message);
        data.push(message);
        dispatcher->wakeUp();
      }
    }
    catch (std::exception& e)
    {
      ROS_ERROR("%s", e.what());
      // use proper exception handling, avoid ros libraries!
    }
  }
}

std::string ReadingThread::getData()
{
  if (data.size() > 0)
  {
    // ROS_INFO_STREAM("ThreadDispatcher getting Data..");
    std::string out(data.front());
    data.pop();
    return out;
  }
  else
  {
    return std::string("");
  }
}

const bool ReadingThread::isQueueEmpty() const { return data.empty(); }
