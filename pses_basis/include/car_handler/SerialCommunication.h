#ifndef SerialCommunication_H
#define SerialCommunication_H

#include <termios.h>
#include <string>
#include <sstream>
#include <unistd.h>
#include <sys/time.h>
#include <serial/serial.h>


class SerialCommunication{
public:
	SerialCommunication(uint32_t baudRate, std::string deviceName) : baudRate(baudRate), deviceName(deviceName) {
	};
	inline bool openConnection() {
		 try{
			serialConnection.setPort("/dev/" + deviceName);
			serialConnection.setBaudrate(baudRate);
			serial::Timeout timeout = serial::Timeout::simpleTimeout(500);
			timeout.inter_byte_timeout = serial::Timeout::max();
			serialConnection.setTimeout(timeout);
			serialConnection.open();
		}
		catch(serial::IOException& e){
			ROS_ERROR("Unable to open serial port");
			return false;
		}
		if(serialConnection.isOpen()){
			ROS_INFO("Serial Port correctly initialized");
			return true;
		}
		return false;
	}
	inline bool receive(std::string& data) {
		if(serialConnection.available()){
			//data = serialConnection.readline(65536, "\x03");
			serialConnection.readline(data, 65536, "\x03");
			data.at(data.size()-1)='\n';
			return true;
			/*
			if(data.size() > 3 && data.at(1) == ':') {
				data = data.substr(2,data.size()-3); //Remove '\n:' at the beginning and '\x03' at the end of readed line
				//ROS_INFO_STREAM("Received: " << data);
				return true;
			}
			*/
		}
		return false;
	}
	inline bool send(const std::string& cmd) {
		std::string send = cmd + "\n";
		ROS_INFO_STREAM("Sent: "<< send);
		return serialConnection.write(send);
	}
	inline bool isOpen() {
		serialConnection.isOpen();
	}
	inline ~SerialCommunication() {
		if(isOpen()) serialConnection.close();
	}
private:
	uint32_t baudRate;
	std::string deviceName;
	serial::Serial serialConnection;
};

#endif