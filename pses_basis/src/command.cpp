#include <pses_basis/command.h>

Command::Command() {}

Command::Command(const Command& other)
    : name(other.name), cmdHasParams(other.cmdHasParams),
      cmdHasResponse(other.cmdHasResponse), respHasParams(other.respHasParams),
      parameterTypes(other.parameterTypes), cmdKeyWords(other.cmdKeyWords),
      cmdParameter(other.cmdParameter), respKeyWords(other.respKeyWords),
      respParameter(other.respParameter),
      commandTemplate(other.commandTemplate),
      responseTemplate(other.responseTemplate)
{
}

Command::Command(const CommandParams& cmdParams)
{
  name = cmdParams.name;
  cmdHasParams = cmdParams.cmdHasParams;
  cmdHasResponse = cmdParams.cmdHasResponse;
  respHasParams = cmdParams.respHasParams;

  parameterTypes = std::unordered_map<std::string, std::string>();
  if (cmdParams.cmdHasParams)
  {
    for (std::pair<std::string, std::string> param : cmdParams.params)
    {
      parameterTypes.insert(param);
    }
  }

  commandTemplate = std::vector<std::pair<int, insertInstruction>>();
  cmdKeyWords = std::vector<std::string>();
  cmdParameter = std::vector<std::string>();

  std::vector<std::string> split;
  boost::split(split, cmdParams.cmd, boost::is_any_of(" "));
  int keyWordCounter = 0;
  int paramCounter = 0;
  for (std::string s : split)
  {
    if (s.at(0) == '$')
    {
      commandTemplate.push_back(
          std::make_pair(paramCounter, &Command::insertCmdParameter));
      cmdParameter.push_back(s.substr(1, std::string::npos));
      paramCounter++;
    }
    else
    {
      commandTemplate.push_back(
          std::make_pair(keyWordCounter, &Command::insertCmdKeyword));
      cmdKeyWords.push_back(s);
      keyWordCounter++;
    }
  }

  responseTemplate = std::vector<std::pair<int, insertInstruction>>();
  respKeyWords = std::vector<std::string>();
  respParameter = std::vector<std::string>();

  if (cmdParams.cmdHasResponse)
  {
    std::vector<std::string> split;
    boost::split(split, cmdParams.response, boost::is_any_of(" "));
    keyWordCounter = 0;
    paramCounter = 0;
    for (std::string s : split)
    {
      if (s.at(0) == '$')
      {
        responseTemplate.push_back(
            std::make_pair(paramCounter, &Command::insertRespParameter));
        respParameter.push_back(s.substr(1, std::string::npos));
        paramCounter++;
      }
      else
      {
        responseTemplate.push_back(
            std::make_pair(keyWordCounter, &Command::insertRespKeyword));
        respKeyWords.push_back(s);
        keyWordCounter++;
      }
    }
  }
}

void Command::generateCommand(const Parameter::ParameterMap& inputParams,
                              std::string& out)
{
  std::stringstream ss = std::stringstream();
  std::string s;
  for (auto component : commandTemplate)
  {
    s = "";
    (this->*component.second)(component.first, inputParams, s);
    ss <<" "<< s;
  }
  out = ss.str().substr(1, std::string::npos);
}

const std::string& Command::getName() const { return name; }

void Command::insertCmdKeyword(const int& index,
                               const Parameter::ParameterMap& input,
                               std::string& out)
{
  out = cmdKeyWords[index];
}
void Command::insertRespKeyword(const int& index,
                                const Parameter::ParameterMap& input,
                                std::string& out)
{
  out = respKeyWords[index];
}

void Command::insertCmdParameter(const int& index,
                                 const Parameter::ParameterMap& input,
                                 std::string& out)
{
  const std::string& param = cmdParameter[index];
  const std::string& type = parameterTypes[param];
  input.getParameterValueAsString(param, out);
}
void Command::insertRespParameter(const int& index,
                                  const Parameter::ParameterMap& input,
                                  std::string& out)
{
  const std::string& param = respParameter[index];
  const std::string& type = parameterTypes[param];
  input.getParameterValueAsString(param, out);
}
