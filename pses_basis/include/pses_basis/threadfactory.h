#ifndef THREADFACTORY_H
#define THREADFACTORY_H

#include <string>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <utility>
#include <vector>

namespace Params
{

struct Syntax
{
  std::string endOfMessage;
  std::string endOfFrame;
  std::string cmdPrefix;
  std::string reqPrefix;
  std::string answerOnCmdPrefix;
  std::string answerOnReqPrefix;
  std::string channelGrpMsgPrefix;
  std::string errorPrefix;
  std::string optionsPrefix;
};

struct DataType
{
  std::string name;
  unsigned int size;
  bool isSigned;
  bool isFloat;
};

struct Command
{
  std::string name;
  bool cmdHasParams;
  std::vector< std::shared_ptr<Type> > params;
  std::string cmd;
  bool cmdHasResponse;
  bool respHasParams;
  std::string response;
};

void operator>>(const YAML::Node& node, Syntax& syntax)
{
  syntax.endOfMessage = node["end_of_message"].as<std::string>();
  syntax.endOfFrame = node["end_of_frame"].as<std::string>();
  syntax.cmdPrefix = node["command_prefix"].as<std::string>();
  syntax.reqPrefix = node["request_prefix"].as<std::string>();
  syntax.answerOnCmdPrefix = node["answer_on_command_prefix"].as<std::string>();
  syntax.answerOnReqPrefix = node["answer_on_request_prefix"].as<std::string>();
  syntax.channelGrpMsgPrefix = node["channel_group_msg_prefix"].as<std::string>();
  syntax.errorPrefix = node["error_prefix"].as<std::string>();
  syntax.optionsPrefix = node["options_prefix"].as<std::string>();
}

void operator>>(const YAML::Node& node, DataType& dataType)
{
  dataType.name = node["name"].as<std::string>();
  dataType.size = node["byte_size"].as<unsigned int>();
  dataType.isSigned = node["signed"].as<bool>();
  dataType.isFloat = node["float"].as<bool>();
}

template< typename T >
class Parameter
{
public:
    Parameter(const std::string& name, const T& data):name(name), m_data(data){}
private:
    std::string name;
    T m_data;
};

}


class ThreadFactory
{
public:
  ThreadFactory(std::string configPath);
  void readDataTypes();
  void readGeneralSyntax();

private:
  std::string configPath;
  std::unordered_map<std::string, Params::DataType> dataTypes;
  Params::Syntax syntax;
};

#endif // THREADFACTORY_H
