#include <pses_ucbridge/threaddispatcher.h>

ThreadDispatcher::ThreadDispatcher(const std::shared_ptr<Syntax>& syntax)
{
  this->syntax = syntax;
  commandResponse = std::queue<std::string>();
  sensorGroupMessage = std::queue<std::string>();
  wakeUpCommunication = false;
  debugMsgEnabled = false;
  rawCommunicationEnabled = false;
  errorCBregistered = false;
  textCBregistered = false;
}

void ThreadDispatcher::startThread()
{
  active = true;
  worker = std::thread(&ThreadDispatcher::workerFunction, this);
  sensorGroupThread->startThread();
  readingThread->startThread();

}

void ThreadDispatcher::stopThread()
{
  readingThread->stopThread();
  sensorGroupThread->stopThread();
  active = false;
  wakeUp();
  worker.join();
}

void ThreadDispatcher::enableDebugMessages(debugCallbackPtr debug)
{
  this->debug = debug;
  debugMsgEnabled = true;
}

void ThreadDispatcher::enableRawCommunication()
{
  rawCommunicationEnabled = true;
}

void ThreadDispatcher::workerFunction()
{
  while (active)
  {
    sleep();
    while (!readingThread->isQueueEmpty() && active)
    {
      std::string data = readingThread->getData();
      // in case of empty string
      if (data.size() < 1)
        continue;
      if (debugMsgEnabled)
        debug(data);
      if (rawCommunicationEnabled)
      {
        continue;
      }
      // check for known prefixes
      if (data.find(syntax->cmdErrorPrefix) == 0)
      {
        if (errorCBregistered)
          error("UC board reported the following command error:\n"+data);
      }
      else if (data.find(syntax->genErrorPrefix) == 0)
      {
        if (errorCBregistered)
          error("UC board reported the following error:\n"+data);
      }
      else if (data.find(syntax->textMsgPrefix) == 0)
      {
        if (textCBregistered)
          text(data);
      }
      else if (data.find(syntax->channelGrpMsgPrefix) == 0)
      {
        sensorGroupMessage.push(data);
        sensorGroupThread->wakeUp();
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

void ThreadDispatcher::registerErrorCallback(debugCallbackPtr error)
{
  this->error = error;
  errorCBregistered = true;
  sensorGroupThread->registerErrorCallback(error);
  readingThread->registerErrorCallback(error);

}
void ThreadDispatcher::registerTextCallback(debugCallbackPtr text)
{
  this->text = text;
  textCBregistered = true;
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
