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
	SerialCommunication(uint32_t baudRate, std::string deviceName) : baudRate(baudRate) {
		this->deviceName = "/dev/"+deviceName;
		modemDevice = -1;
 
		config.c_cflag =  baudRate | CRTSCTS | CS8; 

	};
	inline bool openConnection() {
		modemDevice = open(deviceName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
		if(modemDevice == -1) {
		  return false;
		}
		else {
			tcgetattr(modemDevice,&oldConfig); /* save current serial port settings */
			tcflush(modemDevice, TCIFLUSH);
	        tcsetattr(modemDevice, TCSANOW, &config);
	        std::string text = "?ver\n";
	        write( modemDevice,(char*)text.c_str(),(size_t)text.length() );
			return true;
		}
	}
	inline bool receive(std::string& data) {

	fd_set set;
	FD_ZERO(&set); /* clear the set */
	FD_SET(modemDevice, &set); /* add our file descriptor to the set */

	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = 100000;

	uint32_t bufferSize = 64;
	std::string delimiter = "\x03";
	std::stringstream ss;
	std::string temp = "";

	int rv = select(modemDevice + 1, &set, NULL, NULL, &timeout);

		while(temp.find(delimiter)<0 && rv>0){
			char buffer[bufferSize];
			temp = "";

			read(modemDevice,buffer,bufferSize);
			temp = std::string(buffer);
			ss<<temp;
		}

	if(rv<=0){
		return false;
	}

	data = ss.str();
	return true;

	}
	inline bool send(std::string& cmd) {

	}
	~SerialCommunication() {
		if(modemDevice != -1) {
			tcflush(modemDevice, TCIFLUSH);
			tcsetattr(modemDevice, TCSANOW, &oldConfig);
			close(modemDevice);
		}
	}
private:
	uint32_t baudRate;
	std::string deviceName;
	int modemDevice;
	struct termios config;
	struct termios oldConfig;
};

#endif