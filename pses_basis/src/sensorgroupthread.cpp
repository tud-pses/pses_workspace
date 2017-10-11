#include <pses_basis/sensorgroupthread.h>

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
  // ROS_INFO_STREAM("ReadingThread starting..");
  active = true;
  worker = std::thread(&SensorGroupThread::workerFunction, this);
  // ROS_INFO_STREAM("ReadingThread started..");
}

void SensorGroupThread::stopThread()
{
  // ROS_INFO_STREAM("SensorGroupThread stopping..");
  active = false;
  wakeUp();
  worker.join();
  // ROS_INFO_STREAM("SensorGroupThread stopped..");
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
    // ROS_INFO_STREAM("grp thread: Wakeup..." <<
    // dispatcher->IsMessageQueueEmpty());
    while (!dispatcher->IsMessageQueueEmpty() && active)
    {
      // ros::Time t = ros::Time::now();
      std::string msg, original;
      dispatcher->dequeueSensorGroupMessage(msg);
      original = msg;
      // boost::remove_erase_if(msg, boost::is_any_of(grpMessagePrefix));
      if (msg.size() < 1)
        continue;
      msg.erase(
          boost::remove_if(msg, boost::is_any_of(syntax->channelGrpMsgPrefix)),
          msg.end());
      // temporärer fix
      int indx = msg.find(syntax->answerOnCmdPrefix);
      if (indx == std::string::npos)
        continue;
      msg.at(indx) = ' ';
      // fix end
      boost::trim_left(msg);
      if (msg.size() < 1)
        continue;
      // ROS_INFO_STREAM("grp thread: post erase "<<msg);
      std::vector<std::string> split;
      boost::split(split, msg, boost::is_any_of(" "));
      // ROS_INFO_STREAM("grp thread: split "<<split[0]<<" "<<split[0].size());
      if (split.size() < 2)
        continue;
      if (split[0].size() < 1)
        continue;
      unsigned char grpNum = 0;
      try
      {
        // grpNum = static_cast<unsigned char>(std::stoul(split[0]));
        // ROS_INFO_STREAM(std::stoul(split[0]));
        grpNum = std::stoul(split[0]);
      }
      catch (std::exception& e)
      {
        if (errorCBregistered)
          error("Received sensor group id unreadable: " + split[0]);
        continue;
      }
      // ROS_INFO_STREAM("Post Split...");
      int startIdx = split[0].size();
      // check if requested sensor group exists
      if (sensorGroups.find(grpNum) == sensorGroups.end())
      {
        if (errorCBregistered)
          error("Received sensor group id unknown: " + std::to_string(grpNum));
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
        // ROS_INFO_STREAM(e.what());
        if (errorCBregistered)
          error("An exception occured while processing a group message.\n "
                "Description: " +
                std::string(e.what()) + "\n Message was: " + original);
      }
      // ROS_INFO_STREAM("Round trip t: "<<(ros::Time::now()-t).toSec());
      // ROS_INFO_STREAM("sent ...");
    }
  }
}
