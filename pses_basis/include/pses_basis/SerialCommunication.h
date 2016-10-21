#ifndef SerialCommunication_H
#define SerialCommunication_H

#include <ros/ros.h>
#include <string>
#include <sstream>
#include <serial/serial.h>
#include <vector>
#include <pses_basis/SensorData.h>
#include <pses_basis/Command.h>
#include <limits>

namespace SC{

	static	std::vector<std::string> sensorTable = {"USL", "USF", "USR", "AX", "AY", "AZ", "GX", "GY", "GZ", "HALL_DT", "HALL_DT8", "HALL_CNT", "VSBAT", "VDBAT"}; 

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
		hallSensorDT = 9,
		hallSensorDTFull = 10,
		hallSensorCount = 11,
		batteryVoltageSystem = 12,
		batteryVoltageMotor = 13,
	};

	typedef std::vector<SensorObject> SensorGroup;
    typedef std::vector<SensorGroup> SensorGroups;
}

class SerialCommunication{
public:
	SerialCommunication(uint32_t baudRate, std::string deviceName) : baudRate(baudRate), deviceName(deviceName) {
		kinectOn = false;
	};

	inline bool initUCBoard(){
		std::string status;
		//open serial communication
		if(!openConnection()) return false;
		ros::Duration(0.1).sleep();

		if(!isOpen()) return false;
		//center steering
		if(!setSteeringLevel(0)) return false;
    	if(!receiveAnswerOnRequest(status)) return false;
    	if(!receiveAnswerOnRequest(status)) return false;
    	//activate motor controller
    	if(!setMotorLevel(0)) return false;
    	if(!receiveAnswerOnRequest(status)) return false;

    	//generate sensor groups
    	SC::SensorGroup sg;
    	std::string params;
    	//US-Sensor Group
    	sg = {SC::rangeSensorLeft, SC::rangeSensorFront, SC::rangeSensorRight};
    	params = "~ALL=5";
    	sensorGroups.push_back(sg);
    	if(!setSensorGroup(sg, 1, params)) return false;
    	if(!receiveAnswerOnRequest(status)) return false;
    	//Accel-Sensor Group
    	sg = {SC::accelerometerX, SC::accelerometerY, SC::accelerometerZ};
    	params = "~TS=5 ~AVG=1";
    	sensorGroups.push_back(sg);
    	if(!setSensorGroup(sg, 2, params)) return false;
    	if(!receiveAnswerOnRequest(status)) return false;
    	//Gyro-Sensor Group
    	sg = {SC::gyroscopeX, SC::gyroscopeY, SC::gyroscopeZ};
    	params = "~TS=5 ~AVG=1";
    	sensorGroups.push_back(sg);
    	if(!setSensorGroup(sg, 3, params)) return false;
    	if(!receiveAnswerOnRequest(status)) return false;
    	//Hall-Sensor Group
    	sg = {SC::hallSensorDT, SC::hallSensorDTFull, SC::hallSensorCount};
    	params = "~ALL=5";
    	sensorGroups.push_back(sg);
    	if(!setSensorGroup(sg, 4, params)) return false;
    	if(!receiveAnswerOnRequest(status)) return false;
    	//Misc Group
    	sg = {SC::batteryVoltageSystem, SC::batteryVoltageMotor};
    	params = "~ALL=500";
    	sensorGroups.push_back(sg);
    	if(!setSensorGroup(sg, 5, params)) return false;
    	if(!receiveAnswerOnRequest(status)) return false;

    	//get car id
    	if(!getCarID()) return false;
    	if(!receiveAnswerOnRequest(status)) return false;

    	//start sending sensor information
    	if(!startSensors()) return false;
    	if(!receiveAnswerOnRequest(status)) return false;


	}

	inline bool deactivateUCBoard(){
		if(!isOpen()) return false;
		std::string status;
		//stop sending sensor information
    	if(!stopSensors()) return false;
    	if(!receiveAnswerOnRequest(status)) return false;
    	//reset to initial state
    	if(!reset()) return false;
    	if(!receiveAnswerOnRequest(status)) return false;
	}

	inline bool sendCommand(const pses_basis::Command& cmd) {
		if(!isOpen()) return false;
		std::string debug;

		if(!stopSensors()) return false;
		receiveAnswerOnRequest(debug);
		
		if(!setSteeringLevel(cmd.steering_level)) return false;
		receiveAnswerOnRequest(debug);
		if(!setMotorLevel(cmd.motor_level)) return false;
		receiveAnswerOnRequest(debug);
		if(!kinectOn && cmd.enable_kinect){
			kinectOn = true;
			if(!setKinect(true)) return false;
			receiveAnswerOnRequest(debug);
		}else if(kinectOn && !cmd.enable_kinect){
			kinectOn = false;
			if(!setKinect(false)) return false;
			receiveAnswerOnRequest(debug);
		}

		if(!startSensors()) return false;
    	if(!receiveAnswerOnRequest(debug)) return false;

		return true;
	}

	inline bool getSensorData(pses_basis::SensorData& data){
		if(!isOpen()) return false;
		std::string rawData;
		if(!receive(rawData)) return false;

		//string must identify as vaild sensor group
		int start = rawData.find("##");
		int end = rawData.find("\x03");
		if(start<0 || end<0){
			return false;
		}
		//get group ID
		int groupID = 0;
		int idBegin = start+2;
		int idLength = rawData.find(":")-idBegin;
		try{
				groupID = std::stoi(rawData.substr(idBegin, idLength));
				}catch(std::exception& e){
					return false;
				}
		//remove preamble and trails
		start = idBegin+idLength+1;
		int substrLength = end-start;
		rawData = rawData.substr(idBegin+idLength+1, substrLength);

		//set message meta data
		sensorMessage.header.seq++;
		sensorMessage.header.stamp = ros::Time::now();
		sensorMessage.header.frame_id = "ucBoard";
		//reset wheel measurements
		sensorMessage.hall_sensor_dt = std::numeric_limits<float>::quiet_NaN();
		sensorMessage.hall_sensor_dt_full = std::numeric_limits<float>::quiet_NaN();

		//parse sensor values
		int sensorCount = 0;
		int nextSensor = -1;
		int sensorValue = 0;
		do{
			nextSensor = rawData.find(" | ");
			if(nextSensor<0){
				try{
					sensorValue = std::stoi(rawData);
					assignSensorValue(sensorMessage, sensorValue, sensorGroups[groupID-1][sensorCount]);
				}catch(std::exception& e){
					return false;
				}
				
			}else{
				try{
					sensorValue = std::stoi(rawData.substr(0, nextSensor));
					assignSensorValue(sensorMessage, sensorValue, sensorGroups[groupID-1][sensorCount]);
					sensorCount++;
					start = nextSensor+3;
					substrLength = rawData.size();
					rawData = rawData.substr(start, substrLength);
				}catch(std::exception& e){
					return false;
				}
				
			}

		}while(nextSensor>=0);
		
		data = sensorMessage;

		return true;

	}
	
	inline ~SerialCommunication() {
		if(isOpen()) serialConnection.close();
	}
private:
	uint32_t baudRate;
	std::string deviceName;
	bool kinectOn;
	serial::Serial serialConnection;
	SC::SensorGroups sensorGroups;
	pses_basis::SensorData sensorMessage;

	inline bool openConnection() {
		 try{
			serialConnection.setPort("/dev/" + deviceName);
			serialConnection.setBaudrate(baudRate);
			serial::Timeout timeout = serial::Timeout::simpleTimeout(4);
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

	inline bool isOpen() {
		return serialConnection.isOpen();
	}

	inline bool send(const std::string& cmd) {
		std::string send = cmd + "\n";
		ROS_INFO_STREAM("Sent: "<< send);
		return serialConnection.write(send);
	}

	inline bool receive(std::string& data) {
		if(serialConnection.available()){
			serialConnection.readline(data, 65536, "\x03");
			return true;
		}
		return false;
	}

	inline bool receiveAnswerOnRequest(std::string& answer){
		std::string msg;
		serialConnection.waitReadable();
		if(!receive(msg)) return false;
		if(msg.size()>0){
			//ROS_INFO_STREAM("Answer on request: "<<msg.substr(0,msg.size()-1));
		}
    	answer = msg;
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

	inline bool setKinect(bool enable) {
		std::string out = "!VOUT ";
		if(enable){
			out+="ON";
		}else{
			out+="OFF";
		}
		return send(out);
	} 

	inline bool getCarID(){
		//send request
		std::string giveID = "?ID";
		if(!send(giveID)) return false;
		//read answer
		std::string rawData;
		if(!receiveAnswerOnRequest(rawData)) return false;
		//string must identify as vaild sensor group
		int start = rawData.find(":");
		int end = rawData.find("\x03");
		if(start<0 || end<0){
			return false;
		}

		try{
			sensorMessage.header.car_id = std::stoi(rawData.substr(start+1, end-1));
		}catch(std::exception& e){
			return false;
		}

		return true;

	}

	inline bool setSensorGroup(const std::vector<SC::SensorObject>& sensors, const int numOfGroup, const std::string& parameter){
		if(sensors.size()==0){
			return false;
		}
		std::stringstream ss;
		ss<<"!DAQ GRP "<<numOfGroup<<" "<<parameter;
		for(auto current : sensors){
			ss<<" "<<SC::sensorTable[current];
		}
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

	inline void assignSensorValue(pses_basis::SensorData& data, const int value , const SC::SensorObject& sensor){
		switch(sensor){
			case SC::accelerometerX : 
				data.accelerometer_x = value*8.0/std::pow(2,16)*9.81;
				break;
			case SC::accelerometerY : 
				data.accelerometer_y = value*8.0/std::pow(2,16)*9.81;
				break;
			case SC::accelerometerZ : 
				data.accelerometer_z = value*8.0/std::pow(2,16)*9.81;
				break;
			case SC::gyroscopeX : 
				data.angular_velocity_x = value*1000.0/std::pow(2,16);
				break;
			case SC::gyroscopeY : 
				data.angular_velocity_y = value*1000.0/std::pow(2,16);
				break;
			case SC::gyroscopeZ : 
				data.angular_velocity_z = value*1000.0/std::pow(2,16);
				break;
			case SC::rangeSensorLeft : 
				data.range_sensor_left = value/10000.0;
				break;
			case SC::rangeSensorFront : 
				data.range_sensor_front = value/10000.0;
				break;
			case SC::rangeSensorRight : 
				data.range_sensor_right = value/10000.0;
				break;
			case SC::hallSensorDT : 
				data.hall_sensor_dt = value/10000.0;
				break;
			case SC::hallSensorDTFull : 
				data.hall_sensor_dt_full = value/1000.0;
				break;
			case SC::hallSensorCount : 
				data.hall_sensor_count = value;
				break;
			case SC::batteryVoltageSystem : 
				data.system_battery_voltage = value/1000.0;
				break;
			case SC::batteryVoltageMotor : 
				data.motor_battery_voltage = value/1000.0;
				break;

		}
}
};

#endif