#ifndef COMMUNICATIONCONFIG_H
#define COMMUNICATIONCONFIG_H

#include <pses_basis/command.h>
#include <string>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <utility>
#include <vector>
#include <boost/algorithm/string.hpp>

struct Syntax
{
  std::string endOfMessage;
  std::string endOfFrame;
  std::string answerOnCmdPrefix;
  std::string channelGrpMsgPrefix;
  std::string cmdErrorPrefix;
  std::string genErrorPrefix;
  std::string optionsPrefix;
};

class CommunicationConfig
{
public:
  CommunicationConfig();
  CommunicationConfig(const CommunicationConfig& other);
  CommunicationConfig(std::string configPath);
  //void readDataTypes();
  void readGeneralSyntax();
  void readCommands();
  const Syntax* getSyntax() const;
  const std::unordered_map<std::string, Command>& getCommands() const;


private:
  std::string configPath;
  //std::unordered_map<std::string, std::shared_ptr<Parameter::DataType>> dataTypes;
  std::unordered_map<std::string, Command> commands;
  Syntax syntax;
};

#endif // COMMUNICATIONCONFIG_H
