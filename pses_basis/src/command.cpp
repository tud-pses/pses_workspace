#include <pses_basis/command.h>

Command::Command() {}

Command::Command(const Command& other)
    : name(other.name), cmdHasParams(other.cmdHasParams),
      cmdHasResponse(other.cmdHasResponse), respHasParams(other.respHasParams),
      parameterTypes(other.parameterTypes), cmdKeyWords(other.cmdKeyWords),
      cmdParameter(other.cmdParameter), commandTemplate(other.commandTemplate),
      simpleResponse(other.simpleResponse),
      cmdParameterSet(other.cmdParameterSet),
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
  cmdParameterSet = std::unordered_set<std::string>();

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
      std::string param = s.substr(1, std::string::npos);
      cmdParameter.push_back(param);
      cmdParameterSet.insert(param);
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
  responseTemplate = std::vector<std::pair<std::string, bool>>();
  if (cmdParams.cmdHasResponse)
  {
    simpleResponse = "";
    if (!cmdParams.respHasParams)
    {
      simpleResponse = cmdParams.response;
    }
    else
    {
      std::vector<std::string> split2;
      boost::split(split2, cmdParams.response, boost::is_any_of(" "));
      keyWordCounter = 0;
      paramCounter = 0;
      for (std::string s : split2)
      {
        if (s.at(0) == '$')
        {
          std::string param = s.substr(1, std::string::npos);
          responseTemplate.push_back(std::make_pair(param, true));
        }
        else
        {
          responseTemplate.push_back(std::make_pair(s, false));
        }
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
    ss << " " << s;
  }
  out = ss.str().substr(1, std::string::npos);
}

const bool Command::verifyResponse(const Parameter::ParameterMap& inputParams,
                                   const std::string& response,
                                   Parameter::ParameterMap& outputParams)
{
  outputParams = Parameter::ParameterMap();
  ROS_INFO_STREAM("Response: "<<response);
  // first case -> no response
  if (!cmdHasResponse)
    return true;
  ROS_INFO_STREAM("passed first case");
  // second case -> static response
  std::stringstream ss = std::stringstream();
  std::string s;
  if (!respHasParams)
  {
    return s.compare(simpleResponse) == 0;
  }
  ROS_INFO_STREAM("passed second case");
  // third case -> response contains any amount of parameter
  std::vector<std::string> split;
  boost::split(split, response, boost::is_any_of(" "));
  for (int i = 0; i < split.size(); i++)
  {
    // exit if the response contains more elements than expected
    ROS_INFO_STREAM("component index: "<<i<<" num. of components: "<<responseTemplate.size());
    if (i >= responseTemplate.size())
      return false;
    // check if this component is a variable
    const std::string& param = responseTemplate[i].first;
    ROS_INFO_STREAM("expected component: "<<param<<" got: "<<split[i]);
    if (responseTemplate[i].second)
    {
      // if the param appears in cmd. and resp. -> indicates reply on cmd
      if (cmdParameterSet.find(param) != cmdParameterSet.end())
      {
        std::string expectedValue = "";
        inputParams.getParameterValueAsString(responseTemplate[i].first,
                                              expectedValue);
        if (split[i].compare(expectedValue)!=0)
          return false;
      }
      // if param doesn't appear in cmd. -> indicates request for data
      else
      { 
        outputParams.setParameterValueAsString(param, split[i]);
      }
    }
    // check if keyWord equals the expected response keyWord
    else
    {
      if(split[i].compare(param)!=0) return false;
    }
  }

  return true;
}

const std::string& Command::getName() const { return name; }

void Command::insertCmdKeyword(const int& index,
                               const Parameter::ParameterMap& input,
                               std::string& out)
{
  out = cmdKeyWords[index];
}

void Command::insertCmdParameter(const int& index,
                                 const Parameter::ParameterMap& input,
                                 std::string& out)
{
  const std::string& param = cmdParameter[index];
  const std::string& type = parameterTypes[param];
  input.getParameterValueAsString(param, out);
}
