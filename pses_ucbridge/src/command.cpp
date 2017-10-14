#include <pses_ucbridge/command.h>

Command::Command() {}

Command::Command(const CommandParams& cmdParams,
                 const std::string& cmdResponsePrefix,
                 const std::unordered_map<std::string, std::shared_ptr<CommandOptions>>& options,
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
    if (s.find('$') != std::string::npos)
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
        if (s.find('$') != std::string::npos)
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
    const CommandOptions& cmdopt = *options[s];
    if(cmdopt.opt.size()<1) break;
    std::vector<std::string> split;
    boost::split(split, cmdopt.opt, boost::is_any_of(" ="));
    for (std::string s1 : split)
    {
      if (s1.at(0) == '$')
      {
        std::string value = "";
        inputParams.getParameterValueAsString(s1.substr(1, std::string::npos),
                                              value);
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
                                   const std::string& responseOrig,
                                   Parameter::ParameterMap& outputParams)
{
  // preprocessing
  std::string response = responseOrig;
  response.erase(boost::remove_if(response, boost::is_any_of(cmdResponsePrefix +
                                                             "\x03" + "\n")),
                 response.end());
  boost::trim(response);
  // start
  outputParams = Parameter::ParameterMap();
  // first case -> no response
  if (!cmdHasResponse)
    return true;
  // second case -> static response
  if (!respHasParams)
  {
    return response.compare(simpleResponse) == 0;
  }
  // third case -> response contains any amount of parameter
  std::vector<std::string> split;
  boost::split(split, response, boost::is_any_of(" ="));
  // response contains not enough tokens/params
  if (split.size() < responseTemplate.size())
    return false;
  // continue
  for (int i = 0; i < split.size(); i++)
  {
    // exit if the response contains more elements than expected
    if (i >= responseTemplate.size())
      return false;
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
        // special treatment for string types
        if(parameterTypes[param].compare("string_t")==0){
          std::stringstream ss = std::stringstream();
          for(int n = i; n<split.size(); n++){
            ss<<" "<<split[n];
          }
          std::string out = ss.str().substr(1,std::string::npos);
          outputParams.insertParameter(param, parameterTypes[param], out);
          return true;
        }
        //standard case
        outputParams.insertParameter(param, parameterTypes[param]);
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
                                   const std::string& responseOrig,
                                   Parameter::ParameterMap& outputParams)
{
  // preprocessing
  std::string response = responseOrig;
  response.erase(
      boost::remove_if(response, boost::is_any_of(cmdResponsePrefix)),
      response.end());
  boost::trim(response);
  // start
  outputParams = Parameter::ParameterMap();
  // first case -> no response
  if (!cmdHasResponse)
    return true;
  // second case -> there may be a response of some kind
  std::vector<std::shared_ptr<CommandOptions>> optWithResponse = std::vector<std::shared_ptr<CommandOptions>>();
  for (std::string opt : options)
  {
    if (this->options[opt]->optReturnsParams)
    {
      // gather all options that provoke an additional parameter in the response
      optWithResponse.push_back(this->options[opt]);
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
      if (i >= responseTemplate.size())
      {
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
  for (std::shared_ptr<CommandOptions> opt : optWithResponse)
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
      int dollarIdx = s.find('$');
      int assignIdx = s.find('=');
      if (dollarIdx != std::string::npos && dollarIdx == 0)
      {
        // if the parameter is expected in out and input -> check for equality
        std::string param = s.substr(1, std::string::npos);
        // check if param is string type -> special treatment required
        std::string paramType = "";
        for(auto params : opt->params){
          if(params.first.compare(param)==0){
            paramType = params.second;
            break;
          }
        }
        if(paramType.size()<1) return false;
        if(paramType.compare("string_t")==0){
          std::stringstream ss = std::stringstream();
          for(int n = splitIndex; n<split.size(); n++){
            ss<<" "<<split[n];
          }
          std::string out = ss.str().substr(1,std::string::npos);
          outputParams.insertParameter(param, paramType, out);
          return true;
        }
        // continue with standard case
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
      else if(dollarIdx!=std::string::npos && assignIdx!=std::string::npos){
        std::vector<std::string> asignSplit;
        boost::split(asignSplit, s, boost::is_any_of("="));
        if(split[splitIndex].find('=')==std::string::npos) return false;
        std::vector<std::string> responseSplit;
        boost::split(responseSplit, split[splitIndex], boost::is_any_of("="));

        if(asignSplit.size()!=2) return false;
        if(responseSplit.size()!=2) return false;
        std::string optParam = asignSplit[1].substr(1, std::string::npos);
        std::string optKey =  asignSplit[0].substr(0, std::string::npos);
        std::string respParam = responseSplit[1].substr(0, std::string::npos);
        std::string respKey =  responseSplit[0].substr(0, std::string::npos);
        if(optKey.compare(respKey)!=0) return false;
        std::string optParamValue;
        if(!inputParams.isParamInMap(optParam)) return false;
        inputParams.getParameterValueAsString(optParam, optParamValue);
        if(respParam.compare(optParamValue)!=0) return false;

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
  if (index >= cmdKeyWords.size())
    throw std::out_of_range("Command keyword index out of bounds! Size: " +
                            std::to_string(cmdKeyWords.size()) + " Index: " +
                            std::to_string(index));
  out = cmdKeyWords[index];
}

void Command::insertCmdParameter(const int& index,
                                 const Parameter::ParameterMap& input,
                                 std::string& out)
{
  if (index >= cmdParameter.size())
    throw std::out_of_range("Command parameter index out of bounds! Size: " +
                            std::to_string(cmdParameter.size()) + " Index: " +
                            std::to_string(index));
  const std::string& param = cmdParameter[index];
  if (!input.isParamInMap(param))
    throw std::out_of_range("Command parameter" + param + "not in map.");
  input.getParameterValueAsString(param, out);
}
