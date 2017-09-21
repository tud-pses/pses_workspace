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

const Syntax& CommunicationConfig::getSyntax() const { return syntax; }
