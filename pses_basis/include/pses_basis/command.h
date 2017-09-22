#ifndef COMMAND_H
#define COMMAND_H

#include <string>
#include <utility>
#include <vector>
#include <pses_basis/parameter.h>
#include <boost/algorithm/string.hpp>
#include <ros/ros.h>

struct CommandParams
{
  std::string name;
  bool cmdHasParams;
  // first: name, second: type
  std::vector<std::pair<std::string, std::string>> params;
  // std::vector<std::shared_ptr<Parameter>> params;
  std::string cmd;
  bool cmdHasResponse;
  bool respHasParams;
  std::string response;
};

class Command
{
  typedef void (Command::*insertInstruction)(const int&,
                                             const Parameter::ParameterMap&,
                                             std::string& out);

public:
  Command();
  Command(const Command& other);
  Command(Command&& other) = delete;
  Command(const CommandParams& cmdParams);
  void generateCommand(const Parameter::ParameterMap& inputParams,
                       std::string& out);
  const std::string& getName() const;

private:
  std::string name;
  bool cmdHasParams;
  bool cmdHasResponse;
  bool respHasParams;
  std::unordered_map<std::string, std::string> parameterTypes;
  std::vector<std::string> cmdKeyWords;
  std::vector<std::string> cmdParameter;
  std::vector<std::string> respKeyWords;
  std::vector<std::string> respParameter;
  std::vector<std::pair<int, insertInstruction>> commandTemplate;
  std::vector<std::pair<int, insertInstruction>> responseTemplate;

  void insertCmdKeyword(const int& index, const Parameter::ParameterMap& input,
                        std::string& out);
  void insertRespKeyword(const int& index, const Parameter::ParameterMap& input,
                         std::string& out);
  void insertCmdParameter(const int& index,
                          const Parameter::ParameterMap& input,
                          std::string& out);
  void insertRespParameter(const int& index,
                           const Parameter::ParameterMap& input,
                           std::string& out);
};

#endif // COMMAND_H
