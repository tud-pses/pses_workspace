#include <pses_basis/communicationconfig.h>

CommunicationConfig::CommunicationConfig() {}

CommunicationConfig::CommunicationConfig(const CommunicationConfig& other)
    : configPath(other.configPath)
{
}

CommunicationConfig::CommunicationConfig(std::string configPath)
    : configPath(configPath)
{
  readGeneralSyntax();
  readChannels();
  // readOptions();
  readCommands();
}

// Syntax-struct assign operator
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

// Channel-struct assign operator
void operator>>(const YAML::Node& node,
                std::unordered_map<std::string, Channel>& channels)
{
  Channel ch;
  ch.chName = node["ch_name"].as<std::string>();
  ch.dataType = node["datatype"].as<std::string>();
  ch.conversionNeeded = node["conversion_needed"].as<bool>();
  ch.conversionFactor = 1.0;
  if (node["conversion_factor"].IsScalar() &&
      !node["conversion_factor"].IsNull())
  {
    ch.conversionFactor = node["conversion_factor"].as<double>();
  }
  channels.insert(std::make_pair(ch.chName, ch));
}

// CommandOptions-struct assign operator
void operator>>(const YAML::Node& node,
                std::unordered_map<std::string, CommandOptions>& options)
{
  CommandOptions cmdOpt;
  cmdOpt.optName = node["name"].as<std::string>();
  cmdOpt.optHasParams = node["opt_has_params"].as<bool>();
  cmdOpt.optReturnsParams = node["opt_returns_params"].as<bool>();
  cmdOpt.addsRespToGrps = node["adds_response_to_groups"].as<bool>();
  if (node["opt"].IsScalar() && !node["opt"].IsNull())
  {
    cmdOpt.opt = node["opt"].as<std::string>();
  }
  if (node["response"].IsScalar() && !node["response"].IsNull())
  {
    cmdOpt.response = node["response"].as<std::string>();
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
      cmdOpt.params.push_back(std::make_pair(name, type));
    }
  }
  options.insert(std::make_pair(cmdOpt.optName, cmdOpt));
}

// CommandType-Struct assign operator
void CommunicationConfig::insertCommand(const YAML::Node& node)
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
    cmd.response = node["response"].as<std::string>();
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
  commands.insert(
      std::make_pair(cmd.name, Command(cmd, syntax.answerOnCmdPrefix, &options,
                                       syntax.optionsPrefix)));
}

void CommunicationConfig::readGeneralSyntax()
{
  YAML::Node syntaxYaml = YAML::LoadFile(configPath + "general_syntax.yml");
  syntaxYaml >> syntax;
}

void CommunicationConfig::readChannels()
{
  YAML::Node channelsYaml = YAML::LoadFile(configPath + "channels.yml");

  for (auto node : channelsYaml)
  {
    node >> channels;
  }
}

void CommunicationConfig::readOptions()
{
  YAML::Node commandOpt = YAML::LoadFile(configPath + "command_options.yml");

  for (auto node : commandOpt)
  {
    node >> options;
  }
}

void CommunicationConfig::readCommands()
{
  YAML::Node commandsYaml = YAML::LoadFile(configPath + "commands.yml");

  for (auto node : commandsYaml)
  {
    insertCommand(node);
  }
}

const Syntax* CommunicationConfig::getSyntax() const { return &syntax; }

const std::unordered_map<std::string, Command>&
CommunicationConfig::getCommands() const
{
  return commands;
}
