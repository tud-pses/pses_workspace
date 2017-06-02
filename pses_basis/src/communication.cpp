#include "pses_basis/communication.h"

Communication::Communication() { dispatcher = new ThreadDispatcher("\x03"); }
Communication::~Communication() { delete(dispatcher); }


void Communication::connect()
{
  SerialInterface& si = SerialInterface::instance();
  si.connect();
}

void Communication::startCommunication()
{
  SerialInterface& si = SerialInterface::instance();
  dispatcher->startThread();
  std::string msg = std::string("?ID\n");
  si.send(msg);
  ros::Duration(0.5).sleep();
  msg = "!DAQ GRP 1 ~TS=10 ~AVG AX AY AZ\n";
  si.send(msg);
  ros::Duration(0.5).sleep();
  msg = "!DAQ START\n";
  si.send(msg);
  ros::Duration(10.0).sleep();
}

void Communication::stopCommunication()
{
  SerialInterface& si = SerialInterface::instance();
  std::string msg = "!DAQ STOP\n";
  si.send(msg);
  ros::Duration(0.5).sleep();
  dispatcher->stopThread();
  si.disconnect();
}

void Communication::disconnect()
{
  SerialInterface& si = SerialInterface::instance();
  si.disconnect();
}
