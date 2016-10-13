#ifndef SerialCommunication_H
#define SerialCommunication_H

#include <termios.h>
#include <ros/ros.h>
#include <string>
#include <sstream>
#include <unistd.h>
#include <sys/time.h>
#include <serial/serial.h>
#include <vector>
#include <map>
#include <pses_basis/SensorData.h>
#include <pses_basis/Command.h>

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
		//open serial communication
		if(!openConnection()) return false;
		
		if(!isOpen()){
			return false;
		}
		//center steering
		if(!setSteeringLevel(0)) return false;
    	ros::Duration(0.01).sleep();
    	//activate motor controller
    	if(!setMotorLevel(0)) return false;
    	ros::Duration(0.01).sleep();

    	//generate sensor groups
    	SC::SensorGroup sg;
    	std::string params;
    	//US-Sensor Group
    	sg = {SC::rangeSensorLeft, SC::rangeSensorFront, SC::rangeSensorRight};
    	params = "~ALL=5";
    	sensorGroups.push_back(sg);
    	if(!setSensorGroup(sg, 1, params)) return false;
    	ros::Duration(0.01).sleep();
    	//Accel-Sensor Group
    	sg = {SC::accelerometerX, SC::accelerometerY, SC::accelerometerZ};
    	params = "~SKIP=8";
    	sensorGroups.push_back(sg);
    	if(!setSensorGroup(sg, 2, params)) return false;
    	ros::Duration(0.01).sleep();
    	//Gyro-Sensor Group
    	sg = {SC::gyroscopeX, SC::gyroscopeY, SC::gyroscopeZ};
    	params = "~SKIP=8";
    	sensorGroups.push_back(sg);
    	if(!setSensorGroup(sg, 3, params)) return false;
    	ros::Duration(0.01).sleep();
    	//Hall-Sensor Group
    	sg = {SC::hallSensorDT, SC::hallSensorDTFull, SC::hallSensorCount};
    	params = "~ALL=5";
    	sensorGroups.push_back(sg);
    	if(!setSensorGroup(sg, 4, params)) return false;
    	ros::Duration(0.01).sleep();
    	//Misc Group
    	sg = {SC::batteryVoltageSystem, SC::batteryVoltageMotor};
    	params = "~ALL=500";
    	sensorGroups.push_back(sg);
    	if(!setSensorGroup(sg, 5, params)) return false;
    	ros::Duration(0.01).sleep();

    	//start sending sensor information
    	if(!startSensors()) return false;
    	ros::Duration(0.01).sleep();

	}

	inline bool deactivateUCBoard(){
		if(!isOpen()){
			return false;
		}
		//stop sending sensor information
    	if(!stopSensors()) return false;
    	ros::Duration(0.01).sleep();
    	//reset to initial state
    	if(!reset()) return false;
    	ros::Duration(0.01).sleep();
	}

	inline bool sendCommand(const pses_basis::Command& cmd) {
		if(!isOpen()){
			return false;
		}
		bool state = setSteeringLevel(cmd.steering_level) && setMotorLevel(cmd.motor_level) && (cmd.reset?reset():true);
		if(!kinectOn && cmd.enable_kinect) {
			kinectOn = true;
			return state && setKinect(true);
		}
		else if(kinectOn && !cmd.enable_kinect) {
			kinectOn = false;
			return state && setKinect(false);
		}
		else {
			return state;
		}
	}

	inline bool getSensorData(pses_basis::SensorData& data){
		if(!isOpen()){
			return false;
		}
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
		int idEnd = rawData.find(":")-1;
		try{
				groupID = std::stoi(rawData.substr(idBegin, idEnd));
				}catch(std::exception& e){
					return false;
				}
		//remove preamble and trail
		rawData = rawData.substr(idEnd+2, end-1);

		//set message meta data
		sensorMessage.header.seq++;
		sensorMessage.header.stamp = ros::Time::now();
		sensorMessage.header.frame_id = "ucBoard";
		//reset wheel measurements
		sensorMessage.hall_sensor_dt = -1;
		sensorMessage.hall_sensor_dt_full = -1;

		//ROS_INFO_STREAM("Group: "<< groupID << " String://"<<rawData <<"//");

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
					sensorValue = std::stoi(rawData.substr(0, nextSensor-1));
					assignSensorValue(sensorMessage, sensorValue, sensorGroups[groupID-1][sensorCount]);
					sensorCount++;
					rawData = rawData.substr(nextSensor+3, rawData.size()-1);
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
			serial::Timeout timeout = serial::Timeout::simpleTimeout(10);
			//timeout.inter_byte_timeout = serial::Timeout::max();
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
		std::string out = "!VOUT "+ (enable)?"ON":"OFF";
		return send(out);
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
				data.gyroscope_x = value*1000.0/std::pow(2,16);
				break;
			case SC::gyroscopeY : 
				data.gyroscope_y = value*1000.0/std::pow(2,16);
				break;
			case SC::gyroscopeZ : 
				data.gyroscope_z = value*1000.0/std::pow(2,16);
				break;
			case SC::rangeSensorLeft : 
				data.range_sensor_left = value/1000.0;
				break;
			case SC::rangeSensorFront : 
				data.range_sensor_front = value/1000.0;
				break;
			case SC::rangeSensorRight : 
				data.range_sensor_right = value/1000.0;
				break;
			case SC::hallSensorDT : 
				data.hall_sensor_dt = value/1000.0;
				break;
			case SC::hallSensorDTFull : 
				data.hall_sensor_dt_full = value/1000.0;
				break;
			case SC::hallSensorCount : 
				data.hall_sensor_count = value;
				break;
			case SC::batteryVoltageSystem : 
				data.system_battery_voltage = value/100.0;
				break;
			case SC::batteryVoltageMotor : 
				data.motor_battery_voltage = value/1000.0;
				break;

		}
}
};

#endif