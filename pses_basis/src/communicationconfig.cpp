#include <pses_basis/communicationconfig.h>

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
  // ROS_INFO_STREAM(syntaxYaml);
}

void CommunicationConfig::readDataTypes()
{
  YAML::Node typesYaml = YAML::LoadFile(configPath + "data_types.yml");

  for (auto node : typesYaml)
  {
    node >> dataTypes;
  }
}

void CommunicationConfig::readCommands()
{
  YAML::Node commandsYaml = YAML::LoadFile(configPath + "commands.yml");

  for (auto node : commandsYaml)
  {
    node >> commands;
  }
  for (auto cmd : commands)
  {
    for (std::string param : cmd.second.paramsRaw)
    {
      std::vector<std::string> split;
      boost::split(split, param, boost::is_any_of(":"));
      std::string type = split[0];
      std::string name = split[1];
      cmd.second.params.push_back(
          dataTypes[type]->generateGenericParameter(name, type));
    }
  }
  /*
  auto& prm = commands["Drive Forward"].params[0];
  auto& dt = dataTypes[prm->getType()];
  dt->setGenericParameterData("16", prm);
  //ROS_INFO_STREAM();
  */
}

const Params::Syntax& CommunicationConfig::getSyntax() const { return syntax; }
