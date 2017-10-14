#ifndef SENSORGROUPTHREAD_H
#define SENSORGROUPTHREAD_H

#include <string>
#include <queue>
#include <pses_ucbridge/communicationthread.h>
#include <pses_ucbridge/threaddispatcher.h>
#include <pses_ucbridge/sensorgroup.h>
#include <boost/range/algorithm/remove_if.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <algorithm>

class ThreadDispatcher;

typedef boost::function<void(const std::string&)> errorCallbackPtr;

class SensorGroupThread : public CommunicationThread
{
public:
  SensorGroupThread(std::shared_ptr<Syntax> syntax, ThreadDispatcher* dispatcher, const std::unordered_map<unsigned char, std::shared_ptr<SensorGroup> >& sensorGroups);
  void startThread();
  void stopThread();
  void registerErrorCallback(errorCallbackPtr error);

private:
  ThreadDispatcher* dispatcher;
  std::shared_ptr<Syntax> syntax;
  std::unordered_map<unsigned char, std::shared_ptr<SensorGroup> > sensorGroups;
  errorCallbackPtr error;
  bool errorCBregistered;
  void workerFunction();
};

#endif // SENSORGROUPTHREAD_H
