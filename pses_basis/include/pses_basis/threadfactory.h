#ifndef THREADFACTORY_H
#define THREADFACTORY_H

#include <string>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <utility>

namespace Params
{

struct DataType
{
  std::string name;
  unsigned int size;
  bool isSigned;
  bool isFloat;
};

void operator>>(const YAML::Node& node, DataType& dataType)
{
  dataType.name = node["name"].as<std::string>();
  dataType.size = node["byte_size"].as<unsigned int>();
  dataType.isSigned = node["signed"].as<bool>();
  dataType.isFloat = node["float"].as<bool>();
}

}


class ThreadFactory
{
public:
  ThreadFactory(std::string configPath);
  void readDataTypes();

private:
  std::string configPath;
  std::unordered_map<std::string, Params::DataType> dataTypes;
};

#endif // THREADFACTORY_H
