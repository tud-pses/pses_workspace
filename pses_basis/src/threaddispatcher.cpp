#include "pses_basis/threaddispatcher.h"

ThreadDispatcher::ThreadDispatcher()
{
  readingThread = new ReadingThread("\x03", this);
}

void ThreadDispatcher::startThread(){
  //ROS_INFO_STREAM("ThreadDispatcher starting..");
  active = true;
  worker = std::thread(&ThreadDispatcher::workerFunction,this);
  readingThread->startThread();
  //ROS_INFO_STREAM("ThreadDispatcher started..");
}

void ThreadDispatcher::stopThread(){
  //ROS_INFO_STREAM("ThreadDispatcher stopping..");
  readingThread->stopThread();
  active = false;
  wakeUp();
  worker.join();
  //ROS_INFO_STREAM("ThreadDispatcher stopped..");
}

void ThreadDispatcher::workerFunction(){
  while(active){
    //ROS_INFO_STREAM("ThreadDispatcher sleeping..");
    sleep();
    //ROS_INFO_STREAM("ThreadDispatcher woke up..");
    std::string data = readingThread->getData();
    ROS_INFO_STREAM(data);
  }
}
