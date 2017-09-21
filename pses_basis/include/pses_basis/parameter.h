#ifndef PARAMETER_H
#define PARAMETER_H

#include <string>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <utility>
//#include <vector>
//#include <boost/algorithm/string.hpp>

namespace Parameter
{

class Parameter
{
public:
  Parameter(const std::string& name, const std::string& type)
      : name(name), type(type)
  {
  }
  virtual ~Parameter() {}
  const std::string& getName() const { return name; }
  const std::string& getType() const { return type; }

private:
  std::string name;
  std::string type;
};

template <typename T> class GenericParameter : public Parameter
{
public:
  GenericParameter(const std::string& name, const std::string& type)
      : Parameter(name, type)
  {
  }
  void setData(const T& m_data) { this->m_data = m_data; }
  const T& getData() const { return m_data; }

private:
  T m_data;
};

class ParameterMap
{
public:
  ParameterMap()
  {
    parameters = std::unordered_map<std::string, std::shared_ptr<Parameter>>();
  }
  template <typename T>
  void insertParameter(const std::string& name, const std::string& type,
                       const T& value)
  {
    std::shared_ptr<Parameter> param =
        std::shared_ptr<Parameter>(new GenericParameter<T>(name, type));
    std::dynamic_pointer_cast<GenericParameter<T>>(param)->setData(value);
    parameters.insert(std::make_pair(name, param));
  }
  template <typename T> const T& getParameterValue(const std::string& name)
  {
    return std::dynamic_pointer_cast<GenericParameter<T>>(parameters[name])
        ->getData();
  }
  template <typename T>
  const std::shared_ptr<GenericParameter<T>>&
  getDynamicParameter(const std::string& name)
  {
    return std::dynamic_pointer_cast<GenericParameter<T>>(parameters[name]);
  }
  const std::shared_ptr<Parameter>& getParameter(const std::string& name)
  {
    return parameters[name];
  }

private:
  std::unordered_map<std::string, std::shared_ptr<Parameter>> parameters;
};
/*
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

template <typename T> class GenericDataType : public DataType
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

private:
  stringToDataType conversion;
};

// DataType-Object assign operator
void operator>>(
    const YAML::Node& node,
    std::unordered_map<std::string, std::shared_ptr<DataType>>& dataTypes)
{
  std::string name = node["name"].as<std::string>();
  unsigned int size = node["byte_size"].as<unsigned int>();
  bool isSigned = node["signed"].as<bool>();
  bool isFloat = node["float"].as<bool>();
  bool isString = node["string"].as<bool>();
  std::shared_ptr<DataType> t;
  if (isSigned && !isFloat && !isString)
  {
    t = std::shared_ptr<DataType>(new GenericDataType<int>(
        name, size, isSigned, isFloat, isString, &DataType::strToInt));
  }
  else if (!isSigned && !isFloat && !isString)
  {
    t = std::shared_ptr<DataType>(new GenericDataType<unsigned int>(
        name, size, isSigned, isFloat, isString, &DataType::strToUint));
  }
  else if (isFloat && size == 4 && !isString)
  {
    t = std::shared_ptr<DataType>(new GenericDataType<float>(
        name, size, isSigned, isFloat, isString, &DataType::strToFloat));
  }
  else if (isFloat && size == 8 && !isString)
  {
    t = std::shared_ptr<DataType>(new GenericDataType<double>(
        name, size, isSigned, isFloat, isString, &DataType::strToDouble));
  }
  else
  {
    t = std::shared_ptr<DataType>(new GenericDataType<std::string>(
        name, size, isSigned, isFloat, isString, &DataType::strToStr));
  }
  dataTypes.insert(std::make_pair(name, t));
}
*/
}
#endif // PARAMETER_H
