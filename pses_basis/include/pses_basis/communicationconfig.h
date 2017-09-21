#ifndef COMMUNICATIONCONFIG_H
#define COMMUNICATIONCONFIG_H

#include <pses_basis/parameter.h>
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

struct CommandParams
{
  std::string name;
  bool cmdHasParams;
  std::vector<std::pair<std::string, std::string> > params;
  //std::vector<std::shared_ptr<Parameter>> params;
  std::string cmd;
  bool cmdHasResponse;
  bool respHasParams;
  std::string response;
};

// syntax struct assign operator
void operator>>(const YAML::Node& node, Syntax& syntax)
{
  syntax.endOfMessage = node["end_of_message"].as<std::string>();
  syntax.endOfFrame = node["end_of_frame"].as<std::string>();
  syntax.answerOnCmdPrefix = node["answer_on_command_prefix"].as<std::string>();
  syntax.channelGrpMsgPrefix =
      node["channel_group_msg_prefix"].as<std::string>();
  syntax.cmdErrorPrefix = node["cmd_error_prefix"].as<std::string>();
  syntax.genErrorPrefix = node["gen_error_prefix"].as<std::string>();
  syntax.optionsPrefix = node["options_prefix"].as<std::string>();
}

// CommandType-Struct assign operator
void operator>>(const YAML::Node& node,
                std::unordered_map<std::string, CommandParams>& commands)
{
  CommandParams cmd;
  cmd.name = node["cmd_name"].as<std::string>();
  cmd.cmdHasParams = node["cmd_has_params"].as<bool>();
  cmd.cmdHasResponse = node["cmd_has_response"].as<bool>();
  cmd.respHasParams = node["response_contains_params"].as<bool>();
  if (node["command"].IsScalar() && !node["command"].IsNull())
  {
    cmd.cmd = node["command"].as<std::string>();
  }
  if (node["response"].IsScalar() && !node["response"].IsNull())
  {
    cmd.response = node["command"].as<std::string>();
  }
  const YAML::Node& paramsNode = node["params"];
  if (paramsNode.IsSequence() && paramsNode.size() > 0)
  {
    for (auto item : paramsNode)
    {
      std::string param = item.as<std::string>();
      std::vector<std::string> split;
      boost::split(split, param, boost::is_any_of(":"));
      std::string type = split[0];
      std::string name = split[1];
      cmd.params.push_back(std::make_pair(name, type));

    }
  }
  // instead of inserting the struct -> build a command object directly
  commands.insert(std::make_pair(cmd.name, cmd));
}

class CommunicationConfig
{
public:
  CommunicationConfig();
  CommunicationConfig(const CommunicationConfig& other);
  CommunicationConfig(std::string configPath);
  //void readDataTypes();
  void readGeneralSyntax();
  void readCommands();
  const Syntax& getSyntax() const;


private:
  std::string configPath;
  //std::unordered_map<std::string, std::shared_ptr<Parameter::DataType>> dataTypes;
  std::unordered_map<std::string, CommandParams> commands;
  Syntax syntax;
};

#endif // COMMUNICATIONCONFIG_H
