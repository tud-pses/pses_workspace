#ifndef COMMAND_H
#define COMMAND_H

#include <string>
#include <utility>
#include <vector>
#include <pses_basis/parameter.h>

struct CommandParams
{
  std::string name;
  bool cmdHasParams;
  std::vector<std::pair<std::string, std::string> > params;
  //std::vector<std::shared_ptr<Parameter>> params;
  std::string cmd;
  bool cmdHasResponse;
  bool respHasParams;
  std::string response;
};

class Command
{
public:
  Command(const CommandParams& cmdParams);

};

#endif // COMMAND_H
