#include "pses_basis/threaddispatcher.h"

ThreadDispatcher::ThreadDispatcher():doRead(false)
{

}

void ThreadDispatcher::startReading(std::string delimiter){
  this->delimiter = delimiter;
  doRead = true;
  serialReader = std::thread(&ThreadDispatcher::readFromSerial,this);
}

void ThreadDispatcher::stopReading(){
  doRead = false;
  serialReader.join();
}

void ThreadDispatcher::readFromSerial(){
  SerialInterface& si = SerialInterface::instance();
  while(doRead){
    std::string message;
    try{
      si.read(message, delimiter);
      if(message.size()>1){
        //ROS_INFO_STREAM(message);
      }
    }catch(std::exception& e){
      ROS_ERROR("%s",e.what());
    }
  }
}
