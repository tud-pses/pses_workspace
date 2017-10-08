#include <pses_basis/threaddispatcher.h>

ThreadDispatcher::ThreadDispatcher(const std::shared_ptr<Syntax>& syntax)
{
  this->syntax = syntax;
  commandResponse = std::queue<std::string>();
  sensorGroupMessage = std::queue<std::string>();
  wakeUpCommunication = false;
}

void ThreadDispatcher::startThread()
{
   //ROS_INFO_STREAM("ThreadDispatcher starting..");
  active = true;
  worker = std::thread(&ThreadDispatcher::workerFunction, this);
  sensorGroupThread->startThread();
  readingThread->startThread();

   //ROS_INFO_STREAM("ThreadDispatcher started..");
}

void ThreadDispatcher::stopThread()
{
   //ROS_INFO_STREAM("ThreadDispatcher stopping..");
  readingThread->stopThread();
  sensorGroupThread->stopThread();
  active = false;
  wakeUp();
  worker.join();
   //ROS_INFO_STREAM("ThreadDispatcher stopped..");
}

void ThreadDispatcher::workerFunction()
{
  while (active)
  {
     //ROS_INFO_STREAM("ThreadDispatcher sleeping..");
    sleep();
     //ROS_INFO_STREAM("ThreadDispatcher woke up..");
    while (!readingThread->isQueueEmpty() && active)
    {
      std::string data = readingThread->getData();
      //ROS_INFO_STREAM("Msg in dispatch:\n"<<data);
      // in case of empty string
      if (data.size() < 1)
        continue;
      // check for known prefixes
      //ROS_INFO_STREAM("cmd queue size: "<<commandResponse.size()<<" msg queue size: "<<sensorGroupMessage.size());
      if (data.find(syntax->cmdErrorPrefix) == 0)
      {
        // dispatch cmd-error thread
      }
      else if (data.find(syntax->genErrorPrefix) == 0)
      {
        // dispatch general-error thread
      }
      else if (data.find(syntax->channelGrpMsgPrefix) == 0)
      {
        //ROS_INFO_STREAM("Msg in dispatch pre push: "<<data<<" "<<sensorGroupMessage.size());
        sensorGroupMessage.push(data);
        sensorGroupThread->wakeUp();
        //ROS_INFO_STREAM("Msg in dispatch post push: "<<sensorGroupMessage.front()<<" "<<sensorGroupMessage.size());
      }
      else if (data.find(syntax->answerOnCmdPrefix) == 0)
      {
        commandResponse.push(data);
        if (wakeUpCommunication)
        {
          comCV->notify_one();
          wakeUpCommunication = false;
        }
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

void ThreadDispatcher::setSensorGroupThread(SensorGroupThread* grpThread)
{
  if (!active)
  {
    this->sensorGroupThread = grpThread;
  }
}

void ThreadDispatcher::setCommunicationCondVar(std::condition_variable* condVar)
{
  this->comCV = condVar;
}

void ThreadDispatcher::dequeueResponse(std::string& response)
{
  response = commandResponse.front();
  commandResponse.pop();
}

void ThreadDispatcher::dequeueSensorGroupMessage(std::string& response)
{
  response = sensorGroupMessage.front();
  sensorGroupMessage.pop();
}

const bool ThreadDispatcher::IsResponseQueueEmpty() const
{
  return commandResponse.empty();
}

const bool ThreadDispatcher::IsMessageQueueEmpty() const
{
  return sensorGroupMessage.empty();
}

void ThreadDispatcher::setCommunicationWakeUp(bool wakeUp)
{
  wakeUpCommunication = wakeUp;
}
