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
      std::string bla = "\n";
      si.read(message, bla);
      if (message.size() > 1)
      {
        //ROS_INFO_STREAM("ReadingThread pushing msg and waking ThreadDispatcher.. msg: "<<message);
        data.push(message);
        dispatcher->wakeUp();
      }
    }
    catch (std::exception& e)
    {
      ROS_ERROR("%s", e.what());
      //use proper exception handling, avoid ros libraries!
    }



    // test area:
    /*
    std::string msg = "# 3 123 | 456 | 789";
    data.push(msg);

    msg = "# 2 1123 | 4456 | 7789 | 9877 | 6544 | 3211";
    data.push(msg);
    msg = "# 3 112 | 445 | 778";
    data.push(msg);
    msg = "# 4 123 | 456 | 789";
    data.push(msg);
    msg = "# 5 23 | 56";
    data.push(msg);

    dispatcher->wakeUp();
    ros::Duration(0.005).sleep();
    */
    //end


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
