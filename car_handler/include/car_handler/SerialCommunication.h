#ifndef SerialCommunication_H
#define SerialCommunication_H

#include <termios.h>
#include <string>
#include <sstream>
#include <unistd.h>
#include <sys/time.h>
#include <serial/serial.h>
#include <vector>
#include <map>

namespace SC{

	static	std::vector<std::string> sensorTable = {"USL", "USF", "USR", "AX", "AY", "AZ", "GX", "GY", "GZ"}; 

	enum SensorObject{
		rangeSensorLeft = 0,
		rangeSensorFront = 1,
		rangeSensorRight = 2,
		accelerometerX = 3,
		accelerometerY = 4,
		accelerometerZ = 5,
		gyroscopeX = 6,
		gyroscopeY = 7,
		gyroscopeZ = 8,
	};
}

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

	inline bool send(const std::string& cmd) {
		std::string send = cmd + "\n";
		ROS_INFO_STREAM("Sent: "<< send);
		return serialConnection.write(send);
	}

	inline bool reset(){
		std::string cmd = "!RESET NOW";
		return send(cmd);
	}

	inline bool setSteeringLevel(int level){
		int steering = 0;
		if(level >50){
			steering = -1000;
		}else if(level <-50){
			steering = 1000;
		}else{
			steering = level*(-20);
		}

		std::stringstream ss;
		ss<<"!STEER "<<steering;
		return send(ss.str());
	}

	inline bool setMotorLevel(int level){
		int motor = 0;
		char direction = 'F';
		if(level >20){
			motor = 1000;
			direction = 'F';
		}else if(level <-20){
			motor = 500;
			direction = 'B';
		}else if(level == 0){
			motor = -500;
			direction = 'F';
		}else{
			direction = (level>0)?'F':'B';
			motor = ( (level>0)?50:-25 ) * level;
		}
		
		std::stringstream ss;
		ss<<"!DRV "<<direction<<" "<<motor;
		return send(ss.str());
	}

	inline bool setSensorGroup(const std::vector<SC::SensorObject>& sensors){
		if(sensors.size()==0){
			return false;
		}
		std::stringstream ss;
		ss<<"!DAQ GRP 1 ~ALL=5";
		for(auto current : sensors){
			ss<<" "<<SC::sensorTable[current];
		}
		ss<<" _DTICS";
		return send(ss.str());
	}

	inline bool startSensors(){
		std::string cmd = "!DAQ START";
		return send(cmd);
	}

	inline bool stopSensors(){
		std::string cmd = "!DAQ STOP";
		return send(cmd);
	}

	inline bool receive(std::string& data) {
		if(serialConnection.available()){
			serialConnection.readline(data, 65536, "\x03");
			data.at(data.size()-1)='\n';
			return true;
		}
		return false;
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