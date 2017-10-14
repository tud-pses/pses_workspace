#include <pses_ucbridge/readingthread.h>

ReadingThread::ReadingThread(std::shared_ptr<Syntax> syntax,
                             ThreadDispatcher* dispatcher)
    : syntax(syntax), dispatcher(dispatcher)
{
  data = std::queue<std::string>();
  errorCBregistered = false;
}

void ReadingThread::startThread()
{
  active = true;
  worker = std::thread(&ReadingThread::workerFunction, this);
}

void ReadingThread::stopThread()
{
  active = false;
  wakeUp();
  worker.join();
}

void ReadingThread::registerErrorCallback(errorCallbackPtr error)
{
  this->error = error;
  errorCBregistered = true;
}

void ReadingThread::workerFunction()
{
  SerialInterface& si = SerialInterface::instance();

  while (active)
  {
    std::string message;
    try
    {
      si.read(message, syntax->endOfMessage);
      message.erase(
          boost::remove_if(message, boost::is_any_of(syntax->endOfFrame +
                                                     syntax->endOfMessage)),
          message.end());
      if (message.size() > 1)
      {
        data.push(message);
        dispatcher->wakeUp();
      }
    }
    catch (std::exception& e)
    {
      if (errorCBregistered)
        error("An error occured while reading from the serial connection\n "
              "Description: " +
              std::string(e.what()));
    }
  }
}

std::string ReadingThread::getData()
{
  if (data.size() > 0)
  {
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
