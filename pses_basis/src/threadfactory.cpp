#include <pses_basis/threadfactory.h>

ThreadFactory::ThreadFactory(std::string configPath):configPath(configPath)
{

}

void ThreadFactory::readGeneralSyntax(){
  YAML::Node syntaxYaml = YAML::LoadFile(configPath+"general_syntax.yml");
  syntaxYaml>>syntax;
  ROS_INFO_STREAM(syntaxYaml);
}

void ThreadFactory::readDataTypes(){
  YAML::Node typesYaml = YAML::LoadFile(configPath+"data_types.yml");

  for (auto node : typesYaml)
  {
    node >> dataTypes;
  }
}

void ThreadFactory::readCommands(){
  YAML::Node commandsYaml = YAML::LoadFile(configPath+"commands.yml");

  for (auto node : commandsYaml)
  {
    node >> dataTypes;
  }
}
