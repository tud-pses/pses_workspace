#ifndef UcBoardException_H
#define UcBoardException_H

#include <exception>
#include <string>

class UcBoardException : public std::exception {
public:
	UcBoardException(const std::string& msg):msg(msg){};
	inline const char* what() const throw() {
		return msg.c_str();
	}
private:
	std::string msg;
};

#endif