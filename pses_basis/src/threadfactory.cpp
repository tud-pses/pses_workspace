#include <pses_basis/threadfactory.h>

ThreadFactory::ThreadFactory(std::string configPath):configPath(configPath)
{

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
