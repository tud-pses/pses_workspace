#include <pses_ucbridge/communicationconfig.h>

CommunicationConfig::CommunicationConfig() {}

CommunicationConfig::CommunicationConfig(const CommunicationConfig& other)
    : configPath(other.configPath)
{
}

CommunicationConfig::CommunicationConfig(std::string configPath)
    : configPath(configPath)
{
  readSerialInterfaceConfig();
  readGeneralSyntax();
  readChannels();
  readOptions();
  readCommands();
  readSensorGroups();
}

// Syntax-struct assign operator
void operator>>(const YAML::Node& node, Syntax& syntax)
{
  syntax.endOfMessage = node["end_of_message"].as<std::string>();
  syntax.endOfFrame = node["end_of_frame"].as<std::string>();
  syntax.textMsgPrefix = node["text_msg_prefix"].as<std::string>();
  syntax.answerOnCmdPrefix = node["answer_on_command_prefix"].as<std::string>();
  syntax.channelGrpMsgPrefix =
      node["channel_group_msg_prefix"].as<std::string>();
  syntax.cmdErrorPrefix = node["cmd_error_prefix"].as<std::string>();
  syntax.genErrorPrefix = node["gen_error_prefix"].as<std::string>();
  syntax.optionsPrefix = node["options_prefix"].as<std::string>();
  const YAML::Node& errorAsciiNode = node["grp_errors_ascii"];
  if (errorAsciiNode.IsSequence() && errorAsciiNode.size() > 0){
    for(auto item : errorAsciiNode){
      syntax.grpErrorsAscii.insert(item.as<std::string>());
    }
  }
  const YAML::Node& errorBinaryNode = node["grp_errors_binary"];
  if (errorBinaryNode.IsSequence() && errorBinaryNode.size() > 0){
    for(auto item : errorBinaryNode){
      int c = 0;
      if (item.IsSequence() && item.size() > 0){
        std::string type;
        std::unordered_set<unsigned int> errorSet;
        for(auto item2 : item){
          if(c != 0){
            errorSet.insert(item2.as<unsigned int>());
            c++;
          }else{
            type = item2.as<std::string>();
            c++;
          }
        }
        syntax.grpErrorsBinary.insert(std::make_pair(type, errorSet));
      }
    }
  }

}

// SerialInferfaceConfig-struct assign operator
void operator>>(const YAML::Node& node, SerialInterfaceParams& serialParams)
{
  serialParams.baudRate = node["baudrate"].as<unsigned int>();
  serialParams.deviceTag = node["device_tag"].as<std::string>();
  serialParams.serialTimeout = node["serial_timeout"].as<unsigned int>();
  serialParams.maxLineLength  = node["max_line_length"].as<unsigned int>();
  serialParams.serialDevicesFolder =
      node["serial_devices_folder"].as<std::string>();
}

// Channel-struct assign operator
void operator>>(const YAML::Node& node,
                std::unordered_map<std::string, std::shared_ptr<Channel>>& channels)
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
  channels.insert(std::make_pair(ch.chName, std::make_shared<Channel>(ch)));
}

// CommandOptions-struct assign operator
void operator>>(const YAML::Node& node,
                std::unordered_map<std::string, std::shared_ptr<CommandOptions>>& options)
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
  options.insert(std::make_pair(cmdOpt.optName, std::make_shared<CommandOptions>(cmdOpt)));
}

// CommandObject insertion/creation method
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
  commands.insert(std::make_pair(
      cmd.name,
      std::make_shared<Command>(Command(cmd, syntax.answerOnCmdPrefix, options,
                                        syntax.optionsPrefix))));
}

// SensorGroupObject insertion/creation method
void CommunicationConfig::insertSensorGroup(const YAML::Node& node)
{
  SensorGroupParameter grp;
  grp.grpNumber = node["grp_nr"].as<unsigned int>();
  if (node["name"].IsScalar() && !node["name"].IsNull())
  {
    grp.grpName = node["name"].as<std::string>();
  }
  else
  {
    grp.grpName = "Unspecified Group";
  }
  const YAML::Node& channelNode = node["channels"];
  if (channelNode.IsSequence() && channelNode.size() > 0)
  {
    for (auto item : channelNode)
    {
      std::string channelName = item.as<std::string>();
      // insert check if channel exists later ..
      grp.channels.push_back(channels[channelName]);
    }
  }
  const YAML::Node& optionsNode = node["options"];
  if (optionsNode.IsSequence() && optionsNode.size() > 0)
  {
    for (auto item : optionsNode)
    {
      std::string option = item.as<std::string>();
      std::string name = "";
      std::string params = "";
      if (option.find(':') != std::string::npos)
      {
        std::vector<std::string> split;
        boost::split(split, option, boost::is_any_of(":"));
        name = split[0];
        params = split[1];
      }
      else
      {
        name = option;
      }

      grp.options.push_back(std::make_pair(options[name], params));
    }
  }
  std::string encoding = node["encoding"].as<std::string>();
  if (encoding.compare(SensorGroup::ENCODING_ASCII) == 0 ||
      encoding.compare(SensorGroup::ENCODING_B64) == 0 ||
      encoding.compare(SensorGroup::ENCODING_HEX) == 0)
    grp.encoding = encoding;
  // else: unkonwn encoding for grp nr xy <- should be an error/exception
  sensorGroups.insert(std::make_pair(
      grp.grpNumber, std::make_shared<SensorGroup>(SensorGroup(grp, getSyntax()))));
}

void CommunicationConfig::readSerialInterfaceConfig()
{
  YAML::Node interfaceYaml = YAML::LoadFile(configPath + "serial_interface_config.yml");
  interfaceYaml >> serialParams;
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

void CommunicationConfig::readSensorGroups()
{
  YAML::Node groupsYaml = YAML::LoadFile(configPath + "sensor_groups.yml");

  for (auto node : groupsYaml)
  {
    insertSensorGroup(node);
  }
}

const std::shared_ptr<Syntax> CommunicationConfig::getSyntax() const
{
  return std::make_shared<Syntax>(syntax);
}
const std::shared_ptr<SerialInterfaceParams>
CommunicationConfig::getSerialInterfaceParams() const
{
  return std::make_shared<SerialInterfaceParams>(serialParams);
}

const std::unordered_map<std::string, std::shared_ptr<Command>>&
CommunicationConfig::getCommands() const
{
  return commands;
}

const std::unordered_map<unsigned char, std::shared_ptr<SensorGroup>>&
CommunicationConfig::getSensorGroups() const
{
  return sensorGroups;
}
