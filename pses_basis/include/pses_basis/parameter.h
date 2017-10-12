#ifndef PARAMETER_H
#define PARAMETER_H

#include <string>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <utility>
#include <boost/algorithm/string.hpp>

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
  virtual const bool isTypeSigned() const = 0;
  virtual const bool isTypeArithmetic() const = 0;

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
  const bool isTypeSigned() const { return std::is_signed<T>(); }
  const bool isTypeArithmetic() const { return std::is_arithmetic<T>(); }
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
  const bool isParamInMap(const std::string& name) const
  {
    return parameters.find(name) != parameters.end();
  }

  const int size() const { return parameters.size(); }

  std::string toString() const
  {
    std::stringstream ss = std::stringstream();
    for (auto item : parameters)
    {
      ss << "Name: " << item.first << ", Type: " << item.second->getType()
         << "\n";
    }
    ss << "List size: " << size();
    return ss.str();
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
  void getParameterValue(const std::string& name, T& value) const
  {
    if (parameters.find(name) == parameters.end())
      throw std::out_of_range("Key: \"" + name + "\" not in Map!");
    if (sizeof(value) > parameters.at(name)->getTypeByteSize())
      throw std::invalid_argument("Key: \"" + name + "\" with Type: \"" +
                                  parameters.at(name)->getType() +
                                  "\" doesn't match given variable type.");
    value = std::dynamic_pointer_cast<GenericParameter<T>>(parameters.at(name))
                ->getData();
  }
  template <typename T>
  const std::shared_ptr<GenericParameter<T>>&
  getDynamicParameter(const std::string& name) const
  {
    if (parameters.find(name) == parameters.end())
      throw std::out_of_range("Key: \"" + name + "\" not in Map!");
    return std::dynamic_pointer_cast<GenericParameter<T>>(parameters.at(name));
  }
  const std::shared_ptr<Parameter>& getParameter(const std::string& name) const
  {
    if (parameters.find(name) == parameters.end())
      throw std::out_of_range("Key: \"" + name + "\" not in Map!");
    return parameters.at(name);
  }
  void getParameterValueAsString(const std::string& name,
                                 std::string& out) const
  {
    if (parameters.find(name) == parameters.end())
      throw std::out_of_range("Key: \"" + name + "\" not in Map!");
    const std::string& type = parameters.at(name)->getType();
    int size = getParameter(name)->getTypeByteSize();
    bool isSigned = getParameter(name)->isTypeSigned();
    bool isArithmetic = getParameter(name)->isTypeArithmetic();
    if (type.compare("int8_t") == 0)
    {
      if (size > 1 || !isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match given variable type!");
      char value;
      getParameterValue(name, value);
      out = std::to_string(static_cast<int>(value));
    }
    else if (type.compare("uint8_t") == 0)
    {
      if (size > 1 || isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match given variable type!");
      unsigned char value;
      getParameterValue(name, value);
      out = std::to_string(static_cast<unsigned int>(value));
    }
    else if (type.compare("int16_t") == 0)
    {
      if (size > 2 || !isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match given variable type!");
      short value;
      getParameterValue(name, value);
      out = std::to_string(static_cast<int>(value));
    }
    else if (type.compare("uint16_t") == 0)
    {
      if (size > 2 || isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match given variable type!");
      unsigned short value;
      getParameterValue(name, value);
      out = std::to_string(static_cast<unsigned int>(value));
    }
    else if (type.compare("int32_t") == 0)
    {
      if (size > 4 || !isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match given variable type!");
      int value;
      getParameterValue(name, value);
      out = std::to_string(value);
    }
    else if (type.compare("uint32_t") == 0)
    {
      if (size > 4 || isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match given variable type!");
      unsigned int value;
      getParameterValue(name, value);
      out = std::to_string(value);
    }
    else if (type.compare("int64_t") == 0)
    {
      if (size > 8 || !isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match given variable type!");
      long value;
      getParameterValue(name, value);
      out = std::to_string(value);
    }
    else if (type.compare("uint64_t") == 0)
    {
      if (size > 8 || isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match given variable type!");
      unsigned long value;
      getParameterValue(name, value);
      out = std::to_string(value);
    }
    else if (type.compare("float32_t") == 0)
    {
      if (size > 4 || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match given variable type!");
      float value;
      getParameterValue(name, value);
      out = std::to_string(value);
    }
    else if (type.compare("float64_t") == 0)
    {
      if (size > 8 || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match given variable type!");
      double value;
      getParameterValue(name, value);
      out = std::to_string(value);
    }
    else if (type.compare("string_t") == 0)
    {
      // wie string type mismatch bestimmen ?
      if(isArithmetic) throw std::invalid_argument("Parameter typename \"" + type +
                                                  "\" doesn't match given variable type!");
      getParameterValue(name, out);
    }
    else if (type.compare("string_t[]") == 0)
    {
      if(isArithmetic) throw std::invalid_argument("Parameter typename \"" + type +
                                                  "\" doesn't match given variable type!");
      // wie string type mismatch bestimmen ?
      std::vector<std::string> sArray;
      getParameterValue(name, sArray);
      std::stringstream ss = std::stringstream();
      for (std::string s : sArray)
      {
        ss << " " << s;
      }
      out = ss.str().substr(1, std::string::npos);
    }
    else
    {
      throw std::invalid_argument("Parameter typename \"" + type +
                                  "\" is an unsupported type!");
    }
  }

  void setParameterValueAsString(const std::string& name,
                                 const std::string& input)
  {
    if (parameters.find(name) == parameters.end())
      throw std::out_of_range("Key: \"" + name + "\" not in Map!");
    const std::string& type = parameters.at(name)->getType();
    int size = getParameter(name)->getTypeByteSize();
    bool isSigned = getParameter(name)->isTypeSigned();
    bool isArithmetic = getParameter(name)->isTypeArithmetic();
    if (type.compare("int8_t") == 0)
    {
      if (size > 1 || !isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match stored variable type!");
      int value = std::stoi(input);
      std::dynamic_pointer_cast<GenericParameter<char>>(parameters[name])
          ->setData(static_cast<char>(value));
    }
    else if (type.compare("uint8_t") == 0)
    {
      if (size > 1 || isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match stored variable type!");
      unsigned long value = std::stoul(input);
      std::dynamic_pointer_cast<GenericParameter<unsigned char>>(
          parameters[name])->setData(static_cast<unsigned char>(value));
    }
    else if (type.compare("int16_t") == 0)
    {
      if (size > 2 || !isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match stored variable type!");
      int value = std::stoi(input);
      std::dynamic_pointer_cast<GenericParameter<short>>(parameters[name])
          ->setData(static_cast<short>(value));
    }
    else if (type.compare("uint16_t") == 0)
    {
      if (size > 2 || isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match stored variable type!");
      unsigned long value = std::stoul(input);
      std::dynamic_pointer_cast<GenericParameter<unsigned short>>(
          parameters[name])->setData(static_cast<unsigned short>(value));
    }
    else if (type.compare("int32_t") == 0)
    {
      if (size > 4 || !isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match stored variable type!");
      int value = std::stoi(input);
      std::dynamic_pointer_cast<GenericParameter<int>>(parameters[name])
          ->setData(value);
    }
    else if (type.compare("uint32_t") == 0)
    {
      if (size > 4 || isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match stored variable type!");
      unsigned long value = std::stoul(input);
      std::dynamic_pointer_cast<GenericParameter<unsigned int>>(
          parameters[name])->setData(value);
    }
    else if (type.compare("int64_t") == 0)
    {
      if (size > 8 || !isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match stored variable type!");
      long value = std::stol(input);
      std::dynamic_pointer_cast<GenericParameter<long long>>(parameters[name])
          ->setData(value);
    }
    else if (type.compare("uint64_t") == 0)
    {
      if (size > 8 || isSigned || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match stored variable type!");
      unsigned long value = std::stoul(input);
      std::dynamic_pointer_cast<GenericParameter<unsigned long long>>(
          parameters[name])->setData(value);
    }
    else if (type.compare("float32_t") == 0)
    {
      if (size > 4 || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match stored variable type!");
      float value = std::stof(input);
      std::dynamic_pointer_cast<GenericParameter<float>>(parameters[name])
          ->setData(value);
    }
    else if (type.compare("float64_t") == 0)
    {
      if (size > 8 || !isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match stored variable type!");
      double value = std::stod(input);
      std::dynamic_pointer_cast<GenericParameter<double>>(parameters[name])
          ->setData(value);
    }
    else if (type.compare("string_t") == 0)
    {
      if (isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match stored variable type!");
      std::dynamic_pointer_cast<GenericParameter<std::string>>(parameters[name])
          ->setData(input);
    }
    else if (type.compare("string_t[]") == 0)
    {
      if (isArithmetic)
        throw std::invalid_argument("Parameter typename \"" + type +
                                    "\" doesn't match stored variable type!");
      std::vector<std::string> split;
      boost::split(split, input, boost::is_any_of(" "));

      std::dynamic_pointer_cast<GenericParameter<std::vector<std::string>>>(
          parameters[name])->setData(split);
    }
    else
    {
      throw std::invalid_argument("Parameter typename \"" + type +
                                  "\" is an unsupported type!");
    }
  }

  void insertParameter(const std::string& name, const std::string& type)
  {
    std::shared_ptr<Parameter> param;

    if (type.compare("int8_t") == 0)
    {
      char value = 0;
      param =
          std::shared_ptr<Parameter>(new GenericParameter<char>(name, type));
      std::dynamic_pointer_cast<GenericParameter<char>>(param)->setData(value);
    }
    else if (type.compare("uint8_t") == 0)
    {
      unsigned char value = 0;
      param = std::shared_ptr<Parameter>(
          new GenericParameter<unsigned char>(name, type));
      std::dynamic_pointer_cast<GenericParameter<unsigned char>>(param)
          ->setData(value);
    }
    else if (type.compare("int16_t") == 0)
    {
      short value = 0;
      param =
          std::shared_ptr<Parameter>(new GenericParameter<short>(name, type));
      std::dynamic_pointer_cast<GenericParameter<short>>(param)->setData(value);
    }
    else if (type.compare("uint16_t") == 0)
    {
      unsigned short value = 0;
      param = std::shared_ptr<Parameter>(
          new GenericParameter<unsigned short>(name, type));
      std::dynamic_pointer_cast<GenericParameter<unsigned short>>(param)
          ->setData(value);
    }
    else if (type.compare("int32_t") == 0)
    {
      int value = 0;
      param = std::shared_ptr<Parameter>(new GenericParameter<int>(name, type));
      std::dynamic_pointer_cast<GenericParameter<int>>(param)->setData(value);
    }
    else if (type.compare("uint32_t") == 0)
    {
      unsigned int value = 0;
      param = std::shared_ptr<Parameter>(
          new GenericParameter<unsigned int>(name, type));
      std::dynamic_pointer_cast<GenericParameter<unsigned int>>(param)
          ->setData(value);
    }
    else if (type.compare("int64_t") == 0)
    {
      long value = 0;
      param =
          std::shared_ptr<Parameter>(new GenericParameter<long>(name, type));
      std::dynamic_pointer_cast<GenericParameter<long>>(param)->setData(value);
    }
    else if (type.compare("uint64_t") == 0)
    {
      unsigned long value = 0;
      param = std::shared_ptr<Parameter>(
          new GenericParameter<unsigned long>(name, type));
      std::dynamic_pointer_cast<GenericParameter<unsigned long>>(param)
          ->setData(value);
    }
    else if (type.compare("float32_t") == 0)
    {
      float value = 0.0;
      param =
          std::shared_ptr<Parameter>(new GenericParameter<float>(name, type));
      std::dynamic_pointer_cast<GenericParameter<float>>(param)->setData(value);
    }
    else if (type.compare("float64_t") == 0)
    {
      double value = 0.0;
      param =
          std::shared_ptr<Parameter>(new GenericParameter<double>(name, type));
      std::dynamic_pointer_cast<GenericParameter<double>>(param)
          ->setData(value);
    }
    else if (type.compare("string_t") == 0)
    {
      param = std::shared_ptr<Parameter>(
          new GenericParameter<std::string>(name, type));
      std::dynamic_pointer_cast<GenericParameter<std::string>>(param)
          ->setData("no value");
    }
    else if (type.compare("string_t[]") == 0)
    {
      std::vector<std::string> value;
      param = std::shared_ptr<Parameter>(
          new GenericParameter<std::vector<std::string>>(name, type));
      std::dynamic_pointer_cast<GenericParameter<std::vector<std::string>>>(
          param)->setData(value);
    }
    else
    {
      throw std::invalid_argument("Parameter typename \"" + type +
                                  "\" is an unsupported type!");
    }

    parameters.insert(std::make_pair(name, param));
  }

private:
  std::unordered_map<std::string, std::shared_ptr<Parameter>> parameters;
};
}
#endif // PARAMETER_H
