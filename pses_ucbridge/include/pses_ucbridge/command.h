#ifndef COMMAND_H
#define COMMAND_H

#include <string>
#include <unordered_set>
#include <unordered_map>
#include <utility>
#include <vector>
#include <pses_ucbridge/parameter.h>
#include <boost/algorithm/string.hpp>
#include <boost/range/algorithm/remove_if.hpp>
#include <boost/algorithm/string/trim.hpp>

struct CommandParams
{
  std::string name;
  bool cmdHasParams;
  // first: name, second: type
  std::vector<std::pair<std::string, std::string>> params;
  std::string cmd;
  bool cmdHasResponse;
  bool respHasParams;
  std::string response;
};

struct CommandOptions
{
  std::string optName;
  bool optHasParams;
  // first: name, second: type
  std::vector<std::pair<std::string, std::string>> params;
  std::string opt;
  bool optReturnsParams;
  std::string response;
  bool addsRespToGrps;
};

class Command
{
  typedef void (Command::*insertInstruction)(const int&,
                                             const Parameter::ParameterMap&,
                                             std::string& out);

public:
  Command();
  Command(const CommandParams& cmdParams, const std::string& cmdResponsePrefix,
          const std::unordered_map<std::string, std::shared_ptr<CommandOptions>>& options,
          const std::string& optionsPrefix);
  void generateCommand(const Parameter::ParameterMap& inputParams,
                       std::string& out);
  void generateCommand(const Parameter::ParameterMap& inputParams,
                       const std::vector<std::string>& options,
                       std::string& out);
  const bool verifyResponse(const Parameter::ParameterMap& inputParams,
                            const std::string& responseOrig,
                            Parameter::ParameterMap& outputParams);
  const bool verifyResponse(const Parameter::ParameterMap& inputParams,
                            const std::vector<std::string>& options,
                            const std::string& responseOrig,
                            Parameter::ParameterMap& outputParams);
  const std::string& getName() const;

private:
  std::string name;
  bool cmdHasParams;
  bool cmdHasResponse;
  bool respHasParams;
  std::unordered_map<std::string, std::shared_ptr<CommandOptions>> options;
  std::string optionsPrefix;
  // first: name, second: type
  std::unordered_map<std::string, std::string> parameterTypes;

  std::string cmdResponsePrefix;
  std::unordered_set<std::string> cmdParameterSet;
  std::vector<std::string> cmdKeyWords;
  std::vector<std::string> cmdParameter;
  std::vector<std::pair<int, insertInstruction>> commandTemplate;

  std::string simpleResponse;
  // string contains a keyWord if bool=false, else string contains paramName
  std::vector<std::pair<std::string, bool>> responseTemplate;

  void insertCmdKeyword(const int& index, const Parameter::ParameterMap& input,
                        std::string& out);
  void insertCmdParameter(const int& index,
                          const Parameter::ParameterMap& input,
                          std::string& out);
};

#endif // COMMAND_H
