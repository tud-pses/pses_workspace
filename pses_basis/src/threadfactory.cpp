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
    Params::DataType dataType;
    node >> dataType;
    dataTypes.insert (std::make_pair(dataType.name,dataType));
  }
}
