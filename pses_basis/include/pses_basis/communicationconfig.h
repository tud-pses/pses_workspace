#ifndef COMMUNICATIONCONFIG_H
#define COMMUNICATIONCONFIG_H

#include <pses_basis/command.h>
#include <pses_basis/sensorgroup.h>
#include <string>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <utility>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <unordered_set>
#include <pses_basis/syntax.h>

struct SerialInterfaceParams{
  unsigned int baudRate;
  std::string deviceTag;
  unsigned int serialTimeout;
  unsigned int maxLineLength;
  std::string serialDevicesFolder;
};

class CommunicationConfig
{
public:
  CommunicationConfig();
  CommunicationConfig(const CommunicationConfig& other);
  CommunicationConfig(std::string configPath);
  friend void operator>>(const YAML::Node& node, Syntax& syntax);
  friend void operator>>(const YAML::Node& node, SerialInterfaceParams& serialParams);
  friend void operator>>(const YAML::Node& node,
                         std::unordered_map<std::string, Channel>& channels);
  friend void
  operator>>(const YAML::Node& node,
             std::unordered_map<std::string, CommandOptions>& options);

  const std::shared_ptr<Syntax> getSyntax() const;
  const std::shared_ptr<SerialInterfaceParams> getSerialInterfaceParams() const;
  const std::unordered_map<std::string, std::shared_ptr<Command> >& getCommands() const;
  const std::unordered_map<unsigned char, std::shared_ptr<SensorGroup> >& getSensorGroups() const;

private:
  std::string configPath;
  std::unordered_map<std::string, std::shared_ptr<Command> > commands;
  std::unordered_map<unsigned char, std::shared_ptr<SensorGroup> > sensorGroups;
  std::unordered_map<std::string, Channel> channels;
  std::unordered_map<std::string, CommandOptions> options;
  Syntax syntax;
  SerialInterfaceParams serialParams;

  void insertCommand(const YAML::Node& node);
  void insertSensorGroup(const YAML::Node& node);
  void readSerialInterfaceConfig();
  void readGeneralSyntax();
  void readChannels();
  void readOptions();
  void readCommands();
  void readSensorGroups();
};

#endif // COMMUNICATIONCONFIG_H
