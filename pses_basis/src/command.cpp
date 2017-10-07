#include <pses_basis/command.h>

Command::Command() {}

Command::Command(const CommandParams& cmdParams,
                 const std::string& cmdResponsePrefix,
                 const std::unordered_map<std::string, CommandOptions>& options,
                 const std::string& optionsPrefix)
{
  name = cmdParams.name;
  cmdHasParams = cmdParams.cmdHasParams;
  cmdHasResponse = cmdParams.cmdHasResponse;
  respHasParams = cmdParams.respHasParams;
  this->cmdResponsePrefix = cmdResponsePrefix;
  this->optionsPrefix = optionsPrefix;
  this->options = options;

  parameterTypes = std::unordered_map<std::string, std::string>();
  if (cmdParams.cmdHasParams || cmdParams.respHasParams)
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

void Command::generateCommand(const Parameter::ParameterMap& inputParams,
                              const std::vector<std::string>& optionsList,
                              std::string& out)
{
  std::string cmd = "";
  generateCommand(inputParams, cmd);
  std::stringstream ss = std::stringstream();
  ss << cmd;
  for (std::string s : optionsList)
  {
    bool bla = options.find(s)!=options.end();

    const CommandOptions& cmdopt = options[s];
    std::vector<std::string> split;
    boost::split(split, cmdopt.opt, boost::is_any_of(" ="));
    //ROS_INFO_STREAM("cmd: "<<name<<" opt: "<<cmdopt.opt);
    for (std::string s1 : split)
    {
      //ROS_INFO_STREAM(s1);
      if (s1.at(0) == '$')
      {
        std::string value = "";
        //ROS_INFO_STREAM("fetching param");
        //ROS_INFO_STREAM(inputParams.toString());
        inputParams.getParameterValueAsString(s1.substr(1, std::string::npos),
                                              value);
        //ROS_INFO_STREAM(value);
        ss << "=" << value;
      }
      else
      {
        ss << " " << optionsPrefix << s1;
      }
    }
  }
  out = ss.str();
}

const bool Command::verifyResponse(const Parameter::ParameterMap& inputParams,
                                   const std::string& response,
                                   Parameter::ParameterMap& outputParams)
{
  outputParams = Parameter::ParameterMap();
  ROS_INFO_STREAM("Response: " << response);
  // first case -> no response
  if (!cmdHasResponse)
    return true;
  ROS_INFO_STREAM("passed first case");
  // second case -> static response
  if (!respHasParams)
  {
    return response.compare(simpleResponse) == 0;
  }
  ROS_INFO_STREAM("passed second case");
  // third case -> response contains any amount of parameter
  std::vector<std::string> split;
  boost::split(split, response, boost::is_any_of(" "));
  for (int i = 0; i < split.size(); i++)
  {
    // exit if the response contains more elements than expected
    ROS_INFO_STREAM("component index: " << i << " num. of components: "
                                        << responseTemplate.size());
    if (i >= responseTemplate.size())
      return false;
    // check if this component is a variable
    const std::string& param = responseTemplate[i].first;
    ROS_INFO_STREAM("expected component: " << param << " got: " << split[i]);
    if (responseTemplate[i].second)
    {
      // if the param appears in cmd. and resp. -> indicates reply on cmd
      if (cmdParameterSet.find(param) != cmdParameterSet.end())
      {
        std::string expectedValue = "";
        inputParams.getParameterValueAsString(responseTemplate[i].first,
                                              expectedValue);
        if (split[i].compare(expectedValue) != 0)
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
      if (split[i].compare(param) != 0)
        return false;
    }
  }

  return true;
}

const bool Command::verifyResponse(const Parameter::ParameterMap& inputParams,
                                   const std::vector<std::string>& options,
                                   const std::string& response,
                                   Parameter::ParameterMap& outputParams)
{
  outputParams = Parameter::ParameterMap();
  ROS_INFO_STREAM("Response: " << response);
  // first case -> no response
  if (!cmdHasResponse)
    return true;
  ROS_INFO_STREAM("passed first case");
  // second case -> there may be a response of some kind
  std::vector<CommandOptions*> optWithResponse = std::vector<CommandOptions*>();
  for (std::string opt : options)
  {
    if (this->options[opt].optReturnsParams)
    {
      // gather all options that provoke an additional parameter in the response
      optWithResponse.push_back(&(this->options[opt]));
    }
  }
  std::vector<std::string> split;
  int splitIndex = 0;
  // second case a) -> static response with opt. parameters
  if (!respHasParams)
  {
    // exit if the static response w.o. options doesn't match
    if (simpleResponse.compare(response.substr(0, simpleResponse.size())) != 0)
      return false;
    // extract rest of response and separate on whitespaces
    std::string restResponse =
        response.substr(simpleResponse.size(), std::string::npos);
    boost::split(split, restResponse, boost::is_any_of(" "));
  }
  else
  {
    // second case b) case -> response with params and opt. parameters
    // check every key word and param of the unmodified response
    boost::split(split, response, boost::is_any_of(" "));
    for (int i = 0; i < split.size(); i++)
    {
      // break if the response contains more elements than expected
      if (i >= responseTemplate.size()){
        splitIndex = i;
        break;
      }
      // check if this component is a variable
      const std::string& param = responseTemplate[i].first;
      if (responseTemplate[i].second)
      {
        // if the param appears in cmd. and resp. -> indicates reply on cmd
        if (cmdParameterSet.find(param) != cmdParameterSet.end())
        {
          std::string expectedValue = "";
          inputParams.getParameterValueAsString(responseTemplate[i].first,
                                                expectedValue);
          if (split[i].compare(expectedValue) != 0)
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
        if (split[i].compare(param) != 0)
          return false;
      }
    }
  }
  // at last, check every key word and parameter of the modified response
  for (CommandOptions* opt : optWithResponse)
  {
    // if there are more tokens in the response than expected -> exit
    if (splitIndex >= split.size())
      return false;
    // separate expected option response into tokens at white spaces
    std::vector<std::string> optionSplit;
    boost::split(optionSplit, opt->response, boost::is_any_of(" "));
    for (std::string s : optionSplit)
    {
      // if there are more tokens in the response than expected -> exit
      if (splitIndex >= split.size())
        return false;
      // case: token expected as a parameter
      if (s.at(0) == '$')
      {
        // if the parameter is expected in out and input -> check for equality
        std::string param = s.substr(1, std::string::npos);
        if (outputParams.isParamInMap(param) && inputParams.isParamInMap(param))
        {
          std::string paramValue = "";
          inputParams.getParameterValueAsString(param, paramValue);
          if (split[splitIndex].compare(paramValue) != 0)
            return false;
        }
        // if the parameter is only expected in the output -> insert in output
        else
        {
          outputParams.setParameterValueAsString(param, split[splitIndex]);
        }
      }
      // case: token expected as key word
      else
      {
        if (s.compare(split[splitIndex]) != 0)
          return false;
      }
      splitIndex++;
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
