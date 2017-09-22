#include <pses_basis/communicationconfig.h>

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
                std::unordered_map<std::string, Command>& commands)
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
  // commands.insert(std::make_pair(cmd.name, cmd));
  commands.insert(std::make_pair(cmd.name, Command(cmd)));
  //ROS_INFO_STREAM("Command: "<<cmd.name<<" inserted!");
}

CommunicationConfig::CommunicationConfig() {}

CommunicationConfig::CommunicationConfig(const CommunicationConfig& other)
    : configPath(other.configPath)
{
}

CommunicationConfig::CommunicationConfig(std::string configPath)
    : configPath(configPath)
{
}

void CommunicationConfig::readGeneralSyntax()
{
  YAML::Node syntaxYaml = YAML::LoadFile(configPath + "general_syntax.yml");
  syntaxYaml >> syntax;
}
/*
void CommunicationConfig::readDataTypes()
{
  YAML::Node typesYaml = YAML::LoadFile(configPath + "data_types.yml");

  for (auto node : typesYaml)
  {
    node >> dataTypes;
  }
}
*/
void CommunicationConfig::readCommands()
{
  YAML::Node commandsYaml = YAML::LoadFile(configPath + "commands.yml");

  for (auto node : commandsYaml)
  {
    node >> commands;
  }
}

const Syntax* CommunicationConfig::getSyntax() const { return &syntax; }

const std::unordered_map<std::string, Command>&
CommunicationConfig::getCommands() const
{
  return commands;
}
