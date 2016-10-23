#ifndef PsesUcBoard_H
#define PsesUcBoard_H

#include <ros/ros.h>
#include <string>
#include <sstream>
#include <serial/serial.h>
#include <pses_basis/UcBoardException.h>
#include <pses_basis/InputStack.h>
#include <pses_basis/SensorData.h>
#include <pses_basis/Command.h>

namespace Board {
	// Error Messages
	const std::string CONNECTING_FAILED = "Connection attempt to uc-Board failed!";
	const std::string CONNECTION_NOT_ESTABLISHED = "Connection to uc-Board not established!";
	const std::string CONNECTION_ALREADY_ESTABLISHED = "Connection to uc-Board already established!";
	const std::string TRANSMISSION_FAILED = "Transmission to uc-Board failed!";
	const std::string COMMAND_STEERING_OOB = "Steering level out of bounds => [50, -50]";
	const std::string COMMAND_STEERING_NR = "No valid answer on steering command.";
	const std::string COMMAND_MOTOR_OOB = "Motor level out of bounds => [20, -20]";
	const std::string COMMAND_MOTOR_NR = "No valid answer on motor command.";
	const std::string REQUEST_NO_ID = "No valid answer on request for ID.";
}

class PsesUcBoard{
	public:
	PsesUcBoard(const unsigned int baudRate=921600, const std::string deviceName="ttyUSB0");
	~PsesUcBoard();
	void initUcBoard(const unsigned int serialTimeout=5);
	void setSteering(const int level);
	void setMotor(const int level);
	void deactivateUCBoard();
	void emptyAllStacks();

	private:
	unsigned int baudRate;
	std::string deviceName;
	serial::Serial serialConnection;
	bool connected;
	InputStack* errorStack;
	InputStack* responseStack;
	InputStack* sensorGroupStack;
	InputStack* displayStack;
	int carID;

	void connect(const unsigned int serialTimeout);
	void send(const std::string& msg);
	void receive(std::string& msg);
	void readInputBuffer();
	void sendRequest(const std::string& req, std::string& answer);
	void queryCarID();
};

#endif
