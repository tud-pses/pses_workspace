#ifndef PARAMETER_H
#define PARAMETER_H

#include <string>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <utility>
#include <ros/ros.h>
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
  virtual const int getTypeByteSize() const = 0;

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
  const int getTypeByteSize() const { return sizeof(m_data); }
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
  template <typename T>
  const T& getParameterValue(const std::string& name) const
  {
    return std::dynamic_pointer_cast<GenericParameter<T>>(parameters.at(name))
        ->getData();
  }
  template <typename T>
  const std::shared_ptr<GenericParameter<T>>&
  getDynamicParameter(const std::string& name) const
  {
    return std::dynamic_pointer_cast<GenericParameter<T>>(parameters.at(name));
  }
  const std::shared_ptr<Parameter>& getParameter(const std::string& name) const
  {
    return parameters.at(name);
  }
  void getParameterValueAsString(const std::string& name,
                                 std::string& out) const
  {
    const std::string& type = parameters.at(name)->getType();
    int size = getParameter(name)->getTypeByteSize();
    if (type.compare("int8_t") == 0)
    {
      if (size > 1)
        return; // wichtig hier sollte eine exception wegen type mismatch
                // geworfen werden
      char value = getParameterValue<char>(name);
      out = std::to_string(static_cast<int>(value));
    }
    else if (type.compare("uint8_t") == 0)
    {
      if (size > 1)
        return; // wichtig hier sollte eine exception wegen type mismatch
                // geworfen werden
      unsigned char value = getParameterValue<unsigned char>(name);
      out = std::to_string(static_cast<unsigned int>(value));
    }
    else if (type.compare("int16_t") == 0)
    {
      if (size > 2)
        return; // wichtig hier sollte eine exception wegen type mismatch
                // geworfen werden
      short value = getParameterValue<short>(name);
      out = std::to_string(static_cast<int>(value));
    }
    else if (type.compare("uint16_t") == 0)
    {
      if (size > 2)
        return; // wichtig hier sollte eine exception wegen type mismatch
                // geworfen werden
      unsigned short value = getParameterValue<unsigned short>(name);
      out = std::to_string(static_cast<unsigned int>(value));
    }
    else if (type.compare("int32_t") == 0)
    {
      if (size > 4)
        return; // wichtig hier sollte eine exception wegen type mismatch
                // geworfen werden
      int value = getParameterValue<int>(name);
      out = std::to_string(value);
    }
    else if (type.compare("uint32_t") == 0)
    {
      if (size > 4)
        return; // wichtig hier sollte eine exception wegen type mismatch
                // geworfen werden
      unsigned int value = getParameterValue<unsigned int>(name);
      out = std::to_string(value);
    }
    else if (type.compare("int64_t") == 0)
    {
      if (size > 8)
        return; // wichtig hier sollte eine exception wegen type mismatch
                // geworfen werden
      long value = getParameterValue<long>(name);
      out = std::to_string(value);
    }
    else if (type.compare("uint64_t") == 0)
    {
      if (size > 8)
        return; // wichtig hier sollte eine exception wegen type mismatch
                // geworfen werden
      unsigned long value = getParameterValue<unsigned long>(name);
      out = std::to_string(value);
    }
    else if (type.compare("float32_t") == 0)
    {
      if (size > 4)
        return; // wichtig hier sollte eine exception wegen type mismatch
                // geworfen werden
      float value = getParameterValue<float>(name);
      out = std::to_string(value);
    }
    else if (type.compare("float64_t") == 0)
    {
      if (size > 8)
        return; // wichtig hier sollte eine exception wegen type mismatch
                // geworfen werden
      double value = getParameterValue<double>(name);
      out = std::to_string(value);
    }
    else if (type.compare("string_t") == 0)
    {
      // wie string type mismatch bestimmen ?
      out = getParameterValue<std::string>(name);
    }
    else
    {
      out = "";
    }
  }

  void setParameterValueAsString(const std::string& name,
                                 const std::string& input)
  {
    const std::string& type = parameters.at(name)->getType();
    int size = getParameter(name)->getTypeByteSize();
    if (type.compare("int8_t") == 0)
    {
      if (size > 1)
        return; // wichtig hier sollte eine exception wegen type mismatch
                // geworfen werden
      int value = std::stoi(input);
      std::dynamic_pointer_cast<GenericParameter<char>>(parameters[name])
              ->setData(static_cast<char>(value));
    }
    else if (type.compare("uint8_t") == 0)
    {
      if (size > 1)
        return; // wichtig hier sollte eine exception wegen type mismatch
                // geworfen werden
      unsigned long value = std::stoul(input);
      std::dynamic_pointer_cast<GenericParameter<unsigned char>>(parameters[name])
              ->setData(static_cast<unsigned char>(value));
    }
    else if (type.compare("int16_t") == 0)
    {
      if (size > 2)
        return; // wichtig hier sollte eine exception wegen type mismatch
                // geworfen werden
      int value = std::stoi(input);
      std::dynamic_pointer_cast<GenericParameter<short>>(parameters[name])
              ->setData(static_cast<short>(value));
    }
    else if (type.compare("uint16_t") == 0)
    {
      if (size > 2)
        return; // wichtig hier sollte eine exception wegen type mismatch
                // geworfen werden
      unsigned long value = std::stoul(input);
      std::dynamic_pointer_cast<GenericParameter<unsigned short>>(parameters[name])
              ->setData(static_cast<unsigned short>(value));
    }
    else if (type.compare("int32_t") == 0)
    {
      if (size > 4)
        return; // wichtig hier sollte eine exception wegen type mismatch
                // geworfen werden
      int value = std::stoi(input);
      std::dynamic_pointer_cast<GenericParameter<int>>(parameters[name])
              ->setData(value);
    }
    else if (type.compare("uint32_t") == 0)
    {
      if (size > 4)
        return; // wichtig hier sollte eine exception wegen type mismatch
                // geworfen werden
      unsigned long value = std::stoul(input);
      std::dynamic_pointer_cast<GenericParameter<unsigned int>>(parameters[name])
              ->setData(value);
    }
    else if (type.compare("int64_t") == 0)
    {
      if (size > 8)
        return; // wichtig hier sollte eine exception wegen type mismatch
                // geworfen werden
      long value = std::stol(input);
      std::dynamic_pointer_cast<GenericParameter<long long>>(parameters[name])
              ->setData(value);
    }
    else if (type.compare("uint64_t") == 0)
    {
      if (size > 8)
        return; // wichtig hier sollte eine exception wegen type mismatch
                // geworfen werden
      unsigned long value = std::stoul(input);
      std::dynamic_pointer_cast<GenericParameter<unsigned long long>>(parameters[name])
              ->setData(value);
    }
    else if (type.compare("float32_t") == 0)
    {
      if (size > 4)
        return; // wichtig hier sollte eine exception wegen type mismatch
                // geworfen werden
      float value = std::stof(input);
      std::dynamic_pointer_cast<GenericParameter<float>>(parameters[name])
              ->setData(value);
    }
    else if (type.compare("float64_t") == 0)
    {
      if (size > 8)
        return; // wichtig hier sollte eine exception wegen type mismatch
                // geworfen werden
      double value = std::stod(input);
      std::dynamic_pointer_cast<GenericParameter<double>>(parameters[name])
              ->setData(value);
    }
    else if (type.compare("string_t") == 0)
    {
      std::dynamic_pointer_cast<GenericParameter<std::string>>(parameters[name])
              ->setData(input);
    }
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
