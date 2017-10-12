#ifndef SENSORGROUP_H
#define SENSORGROUP_H

#include <vector>
#include <string>
#include <pses_basis/parameter.h>
#include <pses_basis/command.h>
#include <pses_basis/syntax.h>
#include <boost/range/algorithm/remove_if.hpp>
#include <pses_basis/base64Decoder.h>

struct Channel
{
  std::string chName;
  std::string dataType;
  bool conversionNeeded;
  double conversionFactor;
};

struct SensorGroupParameter
{
  std::string grpName;
  unsigned char grpNumber;
  std::vector<Channel> channels;
  // first: option name, second: option parameter values
  std::vector<std::pair<CommandOptions, std::string>> options;
  std::string encoding;
};

class SensorGroup;

typedef boost::function<void(SensorGroup*)> responseCallback;
typedef boost::function<void(const std::string&)> valueErrorCallbackPtr;

class SensorGroup
{
public:
  SensorGroup();
  SensorGroup(const SensorGroupParameter& sensorParams, std::shared_ptr<Syntax> syntax);
  void processResponse(const std::string& response);
  void setResponseCallback(responseCallback callbackFunction);
  void registerErrorCallback(valueErrorCallbackPtr valueError);
  const std::string& getName() const;
  void createSensorGroupCommand(Command& cmd, std::string& command) const;
  const bool verifyResponseOnComand(Command& cmd,
                                    const std::string& response) const;

  template <typename T>
  inline const bool getChannelValue(const std::string& name, T& value) const
  {
    if (!channelValues.isParamInMap(name))
      return false;
    channelValues.getParameterValue(name, value);
    return true;
  }
  const bool getChannelValueConverted(const std::string& name, double& value) const;

  static const std::string ENCODING_ASCII;
  static const std::string ENCODING_B64;
  static const std::string ENCODING_HEX;

private:
  unsigned char grpNumber;
  std::string grpName;
  std::string responseEncoding;
  std::vector<Channel> channelList;
  std::unordered_map<std::string, Channel> channelMap;
  Parameter::ParameterMap channelValues;
  std::vector<std::string> optionVariableList;
  Parameter::ParameterMap optionValues;
  responseCallback callbackFunction;
  bool callbackRegistered;
  Parameter::ParameterMap cmdInputParams;
  std::vector<std::string> optionsList;
  std::shared_ptr<Syntax> syntax;
  valueErrorCallbackPtr valueError;
  bool valueErrorCBSet;

  void parseResponse(const std::string& response);
};

#endif // SENSORGROUP_H
