#ifndef THREADFACTORY_H
#define THREADFACTORY_H

#include <string>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <utility>
#include <vector>

namespace Params
{

class Parameter
{
public:
  Parameter(const std::string& name) : name(name) {}
  const std::string& getName() const { return name; }

private:
  std::string name;
};

template <typename T> class GenericParameter : Params::Parameter
{
public:
  GenericParameter(const std::string& name, const T& data)
      : Parameter(name), m_data(data)
  {
    s_data = std::string(data);
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

  virtual void stringToDatatype(const std::string& str, T& datatype)
  {
    (this->*conversion)(str, datatype);
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
    t = std::shared_ptr<Params::DataType>(
          new Params::GenericDataType<int>(
              name, size, isSigned, isFloat, isString,
              &Params::DataType::strToInt));
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
}

class ThreadFactory
{
public:
  ThreadFactory(std::string configPath);
  void readDataTypes();
  void readGeneralSyntax();

private:
  std::string configPath;
  std::unordered_map<std::string, std::shared_ptr<Params::DataType> > dataTypes;
  std::unordered_map<std::string,  > dataTypes;
  Params::Syntax syntax;
};

#endif // THREADFACTORY_H
