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

struct Channel
{
  std::string chName;
  std::string dataType;
  bool conversionNeeded;
  double conversionFactor;
};

struct SensorGroups
{
  std::string grpName;
  unsigned char grpNumber;
  // hier ein Channel Objekt ?
  // hier ein Options Objekt ?
};

class CommunicationConfig
{
public:
  CommunicationConfig();
  CommunicationConfig(const CommunicationConfig& other);
  CommunicationConfig(std::string configPath);
  friend void operator>>(const YAML::Node& node, Syntax& syntax);
  friend void operator>>(const YAML::Node& node,
                         std::unordered_map<std::string, Channel>& channels);
  friend void
  operator>>(const YAML::Node& node,
             std::unordered_map<std::string, CommandOptions>& options);

  const Syntax* getSyntax() const;
  const std::unordered_map<std::string, Command>& getCommands() const;

private:
  std::string configPath;
  std::unordered_map<std::string, Command> commands;
  std::unordered_map<std::string, Channel> channels;
  std::unordered_map<std::string, CommandOptions> options;
  Syntax syntax;

  void insertCommand(const YAML::Node& node);
  void readGeneralSyntax();
  void readChannels();
  void readOptions();
  void readCommands();
};

#endif // COMMUNICATIONCONFIG_H
