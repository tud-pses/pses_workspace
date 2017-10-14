#include <pses_ucbridge/sensorgroupthread.h>

SensorGroupThread::SensorGroupThread(
    std::shared_ptr<Syntax> syntax, ThreadDispatcher* dispatcher,
    const std::unordered_map<unsigned char, std::shared_ptr<SensorGroup>>&
        sensorGroups)
    : dispatcher(dispatcher), syntax(syntax), sensorGroups(sensorGroups)
{
  errorCBregistered = false;
}

void SensorGroupThread::startThread()
{
  active = true;
  worker = std::thread(&SensorGroupThread::workerFunction, this);
}

void SensorGroupThread::stopThread()
{
  active = false;
  wakeUp();
  worker.join();
}

void SensorGroupThread::registerErrorCallback(errorCallbackPtr error)
{
  this->error = error;
  errorCBregistered = true;
}

void SensorGroupThread::workerFunction()
{
  while (active)
  {
    sleep();
    while (!dispatcher->IsMessageQueueEmpty() && active)
    {
      std::string msg, original;
      dispatcher->dequeueSensorGroupMessage(msg);
      original = msg;
      if (msg.size() < 1)
        continue;
      // temporary fix for detecting encodings other than ascii
      if (std::count(msg.begin(), msg.end(), '#') == 1)
      {
        msg.erase(boost::remove_if(
                      msg, boost::is_any_of(syntax->channelGrpMsgPrefix)),
                  msg.end());
        if (msg.size() < 3)
          continue;
        int grpNum = base64_decode(msg, 0, 0, 8, false);
        // check if requested sensor group exists
        if (sensorGroups.find(grpNum) == sensorGroups.end())
        {
          if (errorCBregistered)
            error("Received sensor group id unknown: " +
                  std::to_string(grpNum));
          continue;
        }
        boost::trim(msg);
        if (msg.size() < 1)
          continue;
        // call the corresponding group parser and send
        try
        {
          sensorGroups[grpNum]->processResponse(msg);
        }
        catch (std::exception& e)
        {
          if (errorCBregistered)
            error("An exception occured while processing a group message.\n "
                  "Description: " +
                  std::string(e.what()) + "\n Message was: " + original);
        }
      }
      else
      {
        msg.erase(boost::remove_if(
                      msg, boost::is_any_of(syntax->channelGrpMsgPrefix)),
                  msg.end());
        // temporary fix
        int indx = msg.find(syntax->answerOnCmdPrefix);
        if (indx == std::string::npos)
          continue;
        msg.at(indx) = ' ';
        // fix end
        boost::trim_left(msg);
        if (msg.size() < 1)
          continue;
        std::vector<std::string> split;
        boost::split(split, msg, boost::is_any_of(" "));
        if (split.size() < 2)
          continue;
        if (split[0].size() < 1)
          continue;
        unsigned char grpNum = 0;
        try
        {
          grpNum = std::stoul(split[0]);
        }
        catch (std::exception& e)
        {
          if (errorCBregistered)
            error("Received sensor group id unreadable: " + split[0]);
          continue;
        }
        int startIdx = split[0].size();
        // check if requested sensor group exists
        if (sensorGroups.find(grpNum) == sensorGroups.end())
        {
          if (errorCBregistered)
            error("Received sensor group id unknown: " +
                  std::to_string(grpNum));
          continue;
        }
        msg = msg.substr(startIdx, std::string::npos);
        boost::trim(msg);
        if (msg.size() < 1)
          continue;
        // call the corresponding group parser and send
        try
        {
          sensorGroups[grpNum]->processResponse(msg);
        }
        catch (std::exception& e)
        {
          if (errorCBregistered)
            error("An exception occured while processing a group message.\n "
                  "Description: " +
                  std::string(e.what()) + "\n Message was: " + original);
        }
      }
    }
  }
}
