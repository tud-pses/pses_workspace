#ifndef SYNTAX_H
#define SYNTAX_H

#include <string>
#include <unordered_map>
#include <unordered_set>

struct Syntax
{
  std::string endOfMessage;
  std::string endOfFrame;
  std::string textMsgPrefix;
  std::string answerOnCmdPrefix;
  std::string channelGrpMsgPrefix;
  std::string cmdErrorPrefix;
  std::string genErrorPrefix;
  std::string optionsPrefix;
  std::unordered_set<std::string> grpErrorsAscii;
  std::unordered_map<std::string, std::unordered_set<unsigned int> > grpErrorsBinary;
};

#endif // SYNTAX_H
