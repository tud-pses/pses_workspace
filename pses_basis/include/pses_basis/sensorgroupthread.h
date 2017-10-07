#ifndef SENSORGROUPTHREAD_H
#define SENSORGROUPTHREAD_H

#include <string>
#include <queue>
#include <pses_basis/communicationthread.h>
#include <pses_basis/threaddispatcher.h>
#include <pses_basis/sensorgroup.h>
#include <boost/range/algorithm/remove_if.hpp>
#include <boost/algorithm/string/trim.hpp>

class ThreadDispatcher;

class SensorGroupThread : public CommunicationThread
{
public:
  SensorGroupThread(const std::string& grpMessagePrefix, ThreadDispatcher* dispatcher, const std::unordered_map<unsigned char, std::shared_ptr<SensorGroup> >& sensorGroups);
  void startThread();
  void stopThread();

private:
  ThreadDispatcher* dispatcher;
  std::string grpMessagePrefix;
  std::unordered_map<unsigned char, std::shared_ptr<SensorGroup> > sensorGroups;

  void workerFunction();
};

#endif // SENSORGROUPTHREAD_H
