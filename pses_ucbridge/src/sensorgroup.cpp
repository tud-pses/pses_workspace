#include <pses_ucbridge/sensorgroup.h>

const std::string SensorGroup::ENCODING_ASCII = "ASCII";
const std::string SensorGroup::ENCODING_B64 = "B64";
const std::string SensorGroup::ENCODING_HEX = "HEX";

SensorGroup::SensorGroup() {}

SensorGroup::SensorGroup(const SensorGroupParameter& sensorParams,
                         std::shared_ptr<Syntax> syntax)
    : syntax(syntax)
{
  grpNumber = sensorParams.grpNumber;
  grpName = sensorParams.grpName;
  channelList = sensorParams.channels;
  responseEncoding = sensorParams.encoding;
  callbackRegistered = false;
  valueErrorCBSet = false;
  // init channel values map
  channelValues = Parameter::ParameterMap();
  for (std::shared_ptr<Channel> ch : channelList)
  {
    channelValues.insertParameter(ch->chName, ch->dataType);
  }
  // init command params map
  cmdInputParams = Parameter::ParameterMap();
  // insert grp number
  cmdInputParams.insertParameter("grp_nr", "uint8_t", grpNumber);
  // insert channel list
  std::vector<std::string> channels;
  // init channel map
  channelMap = std::unordered_map<std::string, std::shared_ptr<Channel>>();
  for (std::shared_ptr<Channel> ch : channelList)
  {
    channels.push_back(ch->chName);
    channelMap.insert(std::make_pair(ch->chName, ch));
  }
  cmdInputParams.insertParameter("channels", "string_t[]", channels);
  // init options list and adding option parameters
  optionsList = std::vector<std::string>();
  // init value returning options list
  optionVariableList = std::vector<std::string>();
  optionValues = Parameter::ParameterMap();
  for (auto option : sensorParams.options)
  {

    Parameter::ParameterMap tempPm;
    for (auto param : option.first->params)
    {
      tempPm.insertParameter(param.first, param.second);
    }
    // the first part is later needed for the command string
    optionsList.push_back(option.first->optName);
    std::vector<std::string> optionSplit;
    boost::split(optionSplit, option.first->opt, boost::is_any_of(" ="));
    std::vector<std::string> splitParamValues;
    if (option.second.find(',') != std::string::npos)
    {
      boost::split(splitParamValues, option.second, boost::is_any_of(","));
    }
    else
    {
      splitParamValues.push_back(option.second);
    }

    int valueCounter = 0;
    for (std::string optString : optionSplit)
    {
      if (optString.at(0) == '$')
      {
        std::string paramName = optString.substr(1, std::string::npos);
        cmdInputParams.insertParameter(
            paramName, tempPm.getParameter(paramName)->getType());

        if (valueCounter >= splitParamValues.size())
          break; // DANGER! this has to be treated carefully -> not enough
                 // arguments error!

        cmdInputParams.setParameterValueAsString(
            paramName, splitParamValues[valueCounter]);
        valueCounter++;
      }
    }
    // this part is later needed for parsing
    if (option.first->addsRespToGrps)
    {
      std::vector<std::string> split;
      boost::split(split, option.first->response, boost::is_any_of(" "));
      for (std::string s : split)
      {
        if (s.at(0) == '$')
        {
          std::string paramName = s.substr(1, std::string::npos);
          optionVariableList.push_back(paramName);
          optionValues.insertParameter(
              paramName, tempPm.getParameter(paramName)->getType());
        }
      }
    }
  }
}

void SensorGroup::processResponse(const std::string& response)
{
  parseResponse(response);
  if (callbackRegistered)
    callbackFunction(this);
}

void SensorGroup::parseResponse(const std::string& response)
{
  if (responseEncoding.compare(ENCODING_ASCII) == 0)
  {
    std::vector<std::string> split;
    boost::split(split, response, boost::is_any_of(" | "));
    int splitIndex = 0;
    for (std::string s : split)
    {
      s.erase(boost::remove_if(s, boost::is_any_of(" | ")), s.end());
      if (s.size() <= 0)
        continue;
      if (splitIndex >= channelList.size())
        break;

      if (syntax->grpErrorsAscii.find(s) != syntax->grpErrorsAscii.end() &&
          valueErrorCBSet)
      {

        valueError("Group: " + std::to_string(grpNumber) + " ch.: " +
                   channelList[splitIndex]->chName +
                   " had a faulty value!\n Error Code: " + s);
      }
      else
      {
        channelValues.setParameterValueAsString(channelList[splitIndex]->chName,
                                                s);
      }
      splitIndex++;
    }
    // check for options
    if (optionVariableList.size() <= 0)
      return;
    // parse options with returns
    for (; splitIndex < optionVariableList.size(); splitIndex++)
    {
      optionValues.setParameterValueAsString(optionVariableList[splitIndex],
                                             split[splitIndex]);
    }
  }
  else if (responseEncoding.compare(ENCODING_B64) == 0)
  {

    int byteIndex = 1;
    for (int i = 0; i < channelList.size(); i++)
    {
      std::string chName = channelList[i]->chName;
      if (!channelValues.isParamInMap(chName))
        return;

      int typeSize = channelValues.getParameter(chName)->getTypeByteSize();
      bool isSigned = channelValues.getParameter(chName)->isTypeSigned();
      std::string type = channelValues.getParameter(chName)->getType();
      long tempValue = tempValue = base64_decode(
          response, byteIndex, byteIndex + typeSize - 1, typeSize, isSigned);
      // check for value errors;
      if (syntax->grpErrorsBinary.find(type) != syntax->grpErrorsBinary.end())
      {
        // unsigned int errorCode = tempValue;

        if (syntax->grpErrorsBinary[type].find(tempValue) !=
            syntax->grpErrorsBinary[type].end())
        {

          valueError("Group: " + std::to_string(grpNumber) + " ch.: " + chName +
                     " had a faulty value!\n Error Code: " +
                     std::to_string(tempValue));
        }
      }
      std::string val = std::to_string(tempValue);
      channelValues.setParameterValueAsString(channelList[i]->chName, val);
      byteIndex += typeSize;
    }
  }
}

void SensorGroup::setResponseCallback(responseCallback callbackFunction)
{
  this->callbackFunction = callbackFunction;
  callbackRegistered = true;
}

void SensorGroup::registerErrorCallback(valueErrorCallbackPtr valueError)
{
  this->valueError = valueError;
  valueErrorCBSet = true;
}

const std::string& SensorGroup::getName() const { return grpName; }

void SensorGroup::createSensorGroupCommand(Command& cmd,
                                           std::string& command) const
{
  cmd.generateCommand(cmdInputParams, optionsList, command);
}

const bool
SensorGroup::verifyResponseOnComand(Command& cmd,
                                    const std::string& response) const
{
  Parameter::ParameterMap outputParams;
  return cmd.verifyResponse(cmdInputParams, optionsList, response,
                            outputParams);
}

const bool SensorGroup::getChannelValueConverted(const std::string& name,
                                                 double& out) const
{
  if (!channelValues.isParamInMap(name))
    return false;
  const std::string& type = channelValues.getParameter(name)->getType();
  int size = channelValues.getParameter(name)->getTypeByteSize();
  if (type.compare("int8_t") == 0)
  {
    if (size > 1)
      throw std::invalid_argument("Parameter typename \"" + type +
                                  "\" doesn't match given variable type!");
    char value;
    channelValues.getParameterValue(name, value);
    out = value * channelMap.at(name)->conversionFactor;
  }
  else if (type.compare("uint8_t") == 0)
  {
    if (size > 1)
      throw std::invalid_argument("Parameter typename \"" + type +
                                  "\" doesn't match given variable type!");
    unsigned char value;
    channelValues.getParameterValue(name, value);
    out = value * channelMap.at(name)->conversionFactor;
  }
  else if (type.compare("int16_t") == 0)
  {
    if (size > 2)
      throw std::invalid_argument("Parameter typename \"" + type +
                                  "\" doesn't match given variable type!");
    short value;
    channelValues.getParameterValue(name, value);
    out = value * channelMap.at(name)->conversionFactor;
  }
  else if (type.compare("uint16_t") == 0)
  {
    if (size > 2)
      throw std::invalid_argument("Parameter typename \"" + type +
                                  "\" doesn't match given variable type!");
    unsigned short value;
    channelValues.getParameterValue(name, value);
    out = value * channelMap.at(name)->conversionFactor;
  }
  else if (type.compare("int32_t") == 0)
  {
    if (size > 4)
      throw std::invalid_argument("Parameter typename \"" + type +
                                  "\" doesn't match given variable type!");
    int value;
    channelValues.getParameterValue(name, value);
    out = value * channelMap.at(name)->conversionFactor;
  }
  else if (type.compare("uint32_t") == 0)
  {
    if (size > 4)
      throw std::invalid_argument("Parameter typename \"" + type +
                                  "\" doesn't match given variable type!");
    unsigned int value;
    channelValues.getParameterValue(name, value);
    out = value * channelMap.at(name)->conversionFactor;
  }
  else if (type.compare("int64_t") == 0)
  {
    if (size > 8)
      throw std::invalid_argument("Parameter typename \"" + type +
                                  "\" doesn't match given variable type!");
    long value;
    channelValues.getParameterValue(name, value);
    out = value * channelMap.at(name)->conversionFactor;
  }
  else if (type.compare("uint64_t") == 0)
  {
    if (size > 8)
      throw std::invalid_argument("Parameter typename \"" + type +
                                  "\" doesn't match given variable type!");
    unsigned long value;
    channelValues.getParameterValue(name, value);
    out = value * channelMap.at(name)->conversionFactor;
  }
  else if (type.compare("float32_t") == 0)
  {
    if (size > 4)
      throw std::invalid_argument("Parameter typename \"" + type +
                                  "\" doesn't match given variable type!");
    float value;
    channelValues.getParameterValue(name, value);
    out = value * channelMap.at(name)->conversionFactor;
  }
  else if (type.compare("float64_t") == 0)
  {
    if (size > 8)
      throw std::invalid_argument("Parameter typename \"" + type +
                                  "\" doesn't match given variable type!");
    double value;
    channelValues.getParameterValue(name, value);
    out = value * channelMap.at(name)->conversionFactor;
  }
  else
    return false;
}
