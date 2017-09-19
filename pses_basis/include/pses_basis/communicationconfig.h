#ifndef COMMUNICATIONCONFIG_H
#define COMMUNICATIONCONFIG_H

#include <string>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <utility>
#include <vector>
#include <boost/algorithm/string.hpp>

namespace Params
{

class Parameter
{
public:
  Parameter(const std::string& name, const std::string& type)
      : name(name), type(type)
  {
  }
  const std::string& getName() const { return name; }
  const std::string& getType() const { return type; }

private:
  std::string name;
  std::string type;
};

template <typename T> class GenericParameter : public Params::Parameter
{
public:
  GenericParameter(const std::string& name, const std::string& type)
      : Parameter(name, type)
  {
  }
  void setData(const T& m_data, const std::string& s_data)
  {
    this->m_data = m_data;
    this->s_data = s_data;
  }
  const T& getData() const { return m_data; }
  const std::string& toString() const { return s_data; }

private:
  T m_data;
  std::string s_data;
};

class DataType
{
public:
  DataType(const std::string& name, const unsigned int size,
           const bool isSigned, const bool isFloat, const bool isString)
      : name(name), size(size), isSigned(isSigned), isFloat(isFloat),
        isString(isString)
  {
  }
  const std::string& getName() const { return name; }
  virtual void stringToDatatype(const std::string& str, std::string& datatype)
  {
  }
  virtual void stringToDatatype(const std::string& str, int& datatype) {}
  virtual void stringToDatatype(const std::string& str, unsigned int& datatype)
  {
  }
  virtual void stringToDatatype(const std::string& str, float& datatype) {}
  virtual void stringToDatatype(const std::string& str, double& datatype) {}

  void strToStr(const std::string& str, std::string& conv) { conv = str; }
  void strToInt(const std::string& str, int& conv) { conv = std::stol(str); }
  void strToUint(const std::string& str, unsigned int& conv)
  {
    conv = std::stoul(str);
  }
  void strToFloat(const std::string& str, float& conv)
  {
    conv = std::stof(str);
  }
  void strToDouble(const std::string& str, double& conv)
  {
    conv = std::stod(str);
  }
  virtual std::shared_ptr<Params::Parameter>
  generateGenericParameter(const std::string& name, const std::string& type)
  {
  }
  virtual void
  setGenericParameterData(const std::string& data,
                          std::shared_ptr<Params::Parameter>& param)
  {
  }

private:
  std::string name;
  unsigned int size;
  bool isSigned;
  bool isFloat;
  bool isString;
};

template <typename T> class GenericDataType : public Params::DataType
{
public:
  typedef void (GenericDataType::*stringToDataType)(const std::string&, T&);

  GenericDataType(const std::string& name, const unsigned int size,
                  const bool isSigned, const bool isFloat, const bool isString,
                  stringToDataType f)
      : DataType(name, size, isSigned, isFloat, isString)
  {
    conversion = f;
  }

  void stringToDatatype(const std::string& str, T& datatype)
  {
    (this->*conversion)(str, datatype);
  }

  std::shared_ptr<Params::Parameter>
  generateGenericParameter(const std::string& name, const std::string& type)
  {
    return std::shared_ptr<Params::Parameter>(
        new Params::GenericParameter<T>(name, type));
  }

  void setGenericParameterData(const std::string& data,
                               std::shared_ptr<Params::Parameter>& param)
  {
    T convData;
    (this->*conversion)(data, convData);
    //((Params::GenericParameter<T>)(*param)).setData(convData,data);
    // ROS_INFO_STREAM(convData);
  }

private:
  stringToDataType conversion;
};

struct Syntax
{
  std::string endOfMessage;
  std::string endOfFrame;
  std::string cmdPrefix;
  std::string reqPrefix;
  std::string answerOnCmdPrefix;
  std::string answerOnReqPrefix;
  std::string channelGrpMsgPrefix;
  std::string errorPrefix;
  std::string optionsPrefix;
};

struct Command
{
  std::string name;
  bool cmdHasParams;
  std::vector<std::string> paramsRaw;
  std::vector<std::shared_ptr<Params::Parameter>> params;
  std::string cmd;
  bool cmdHasResponse;
  bool respHasParams;
  std::string response;
};

// syntax struct assign operator
void operator>>(const YAML::Node& node, Syntax& syntax)
{
  syntax.endOfMessage = node["end_of_message"].as<std::string>();
  syntax.endOfFrame = node["end_of_frame"].as<std::string>();
  syntax.cmdPrefix = node["command_prefix"].as<std::string>();
  syntax.reqPrefix = node["request_prefix"].as<std::string>();
  syntax.answerOnCmdPrefix = node["answer_on_command_prefix"].as<std::string>();
  syntax.answerOnReqPrefix = node["answer_on_request_prefix"].as<std::string>();
  syntax.channelGrpMsgPrefix =
      node["channel_group_msg_prefix"].as<std::string>();
  syntax.errorPrefix = node["error_prefix"].as<std::string>();
  syntax.optionsPrefix = node["options_prefix"].as<std::string>();
}

// DataType-Object assign operator
void
operator>>(const YAML::Node& node,
           std::unordered_map<std::string, std::shared_ptr<Params::DataType>>&
               dataTypes)
{
  std::string name = node["name"].as<std::string>();
  unsigned int size = node["byte_size"].as<unsigned int>();
  bool isSigned = node["signed"].as<bool>();
  bool isFloat = node["float"].as<bool>();
  bool isString = node["string"].as<bool>();
  std::shared_ptr<Params::DataType> t;
  if (isSigned && !isFloat && !isString)
  {
    t = std::shared_ptr<Params::DataType>(new Params::GenericDataType<int>(
        name, size, isSigned, isFloat, isString, &Params::DataType::strToInt));
  }
  else if (!isSigned && !isFloat && !isString)
  {
    t = std::shared_ptr<Params::DataType>(
        new Params::GenericDataType<unsigned int>(
            name, size, isSigned, isFloat, isString,
            &Params::DataType::strToUint));
  }
  else if (isFloat && size == 4 && !isString)
  {
    t = std::shared_ptr<Params::DataType>(new Params::GenericDataType<float>(
        name, size, isSigned, isFloat, isString,
        &Params::DataType::strToFloat));
  }
  else if (isFloat && size == 8 && !isString)
  {
    t = std::shared_ptr<Params::DataType>(new Params::GenericDataType<double>(
        name, size, isSigned, isFloat, isString,
        &Params::DataType::strToDouble));
  }
  else
  {
    t = std::shared_ptr<Params::DataType>(
        new Params::GenericDataType<std::string>(name, size, isSigned, isFloat,
                                                 isString,
                                                 &Params::DataType::strToStr));
  }
  dataTypes.insert(std::make_pair(name, t));
}

// CommandType-Object assign operator
void operator>>(const YAML::Node& node,
                std::unordered_map<std::string, Params::Command>& commands)
{
  Params::Command cmd;
  cmd.name = node["cmd_name"].as<std::string>();
  cmd.cmdHasParams = node["cmd_has_params"].as<bool>();
  cmd.cmdHasResponse = node["cmd_has_response"].as<bool>();
  cmd.respHasParams = node["response_contains_params"].as<bool>();
  if (node["command"].IsScalar() && !node["command"].IsNull())
  {
    cmd.cmd = node["command"].as<std::string>();
  }
  if (node["response"].IsScalar() && !node["response"].IsNull())
  {
    cmd.response = node["command"].as<std::string>();
  }
  const YAML::Node& paramsNode = node["params"];
  if (paramsNode.IsSequence() && paramsNode.size() > 0)
  {
    for (auto item : paramsNode)
    {
      std::string param = item.as<std::string>();
      cmd.paramsRaw.push_back(param);
      /*
      std::vector<std::string> split;
      boost::split(split, param, boost::is_any_of(":"));
      //ROS_INFO_STREAM(split[0]);
      std::string type = split[0];
      std::string name = split[1];
      if(type.find("uint")!=-1){
        cmd.params.push_back(std::shared_ptr<Params::Parameter>(new
      Params::GenericParameter<float>(name, type, dataTypes[type])));
      }else if(type.find("int")!=-1){

      }else if(type.find("float32")!=-1){

      }else if(type.find("float64")!=-1){

      }else{

      }
      */
    }
  }

  commands.insert(std::make_pair(cmd.name, cmd));
}
}

class CommunicationConfig
{
public:
  CommunicationConfig();
  CommunicationConfig(const CommunicationConfig& other);
  CommunicationConfig(std::string configPath);
  void readDataTypes();
  void readGeneralSyntax();
  void readCommands();
  const Params::Syntax& getSyntax() const;

private:
  std::string configPath;
  std::unordered_map<std::string, std::shared_ptr<Params::DataType>> dataTypes;
  std::unordered_map<std::string, Params::Command> commands;
  Params::Syntax syntax;
};

#endif // COMMUNICATIONCONFIG_H
