#include <pses_basis/PsesUcBoard.h>

PsesUcBoard::PsesUcBoard(const unsigned int baudRate, const std::string deviceName) : baudRate(baudRate), deviceName(deviceName){
	connected = false;
	errorStack = new InputStack(10);
	responseStack = new InputStack(20);
	sensorGroupStack = new InputStack(20);
	displayStack = new InputStack(10);
	carID = -1;
}
PsesUcBoard::~PsesUcBoard() {
	delete errorStack;
	delete responseStack;
	delete sensorGroupStack;
	delete displayStack;

	if(connected){
		serialConnection.close();
	}
}
void PsesUcBoard::initUcBoard(const unsigned int serialTimeout){
	connect(serialTimeout);
	setSteering(0);
	setMotor(0);
	queryCarID();
	//generate sensor groups
    Board::SensorGroup sg;
    std::string params;
    //US-Sensor Group
    sg = {Board::rangeSensorLeft, Board::rangeSensorFront, Board::rangeSensorRight};
    params = "~ALL=5";
    sensorGroups.push_back(sg);
    setSensorGroup(sg, 1, params);
    //Accel-Sensor Group
    sg = {Board::accelerometerX, Board::accelerometerY, Board::accelerometerZ};
    params = "~SKIP=8";
    sensorGroups.push_back(sg);
    setSensorGroup(sg, 2, params);
    //Gyro-Sensor Group
    sg = {Board::gyroscopeX, Board::gyroscopeY, Board::gyroscopeZ};
    params = "~SKIP=8";
    sensorGroups.push_back(sg);
    setSensorGroup(sg, 3, params);
    //Hall-Sensor Group
    sg = {Board::hallSensorDT, Board::hallSensorDTFull, Board::hallSensorCount};
    params = "~ALL=5";
    sensorGroups.push_back(sg);
    setSensorGroup(sg, 4, params);
    //Misc Group
    sg = {Board::batteryVoltageSystem, Board::batteryVoltageMotor};
    params = "~ALL=500";
    sensorGroups.push_back(sg);
    setSensorGroup(sg, 5, params);

    startSensors();
}
void PsesUcBoard::setSteering(const int level){
	if(level > 50 || level <-50){
		throw UcBoardException(Board::COMMAND_STEERING_OOB);
	}
	std::stringstream valueStream;
	std::stringstream checkStream;
	valueStream << level*(-20);
	checkStream << level*(-10);
	std::string value = valueStream.str();
	std::string check = checkStream.str();
	std::string command = "!STEER " + value;
	std::string answer;
	ros::Time start = ros::Time::now();
	do{
		sendRequest(command, answer);
	}while(answer.find(check)==-1 && (ros::Time::now()-start).toSec()<=0.1);
	if(answer.find(check)==-1){
		throw UcBoardException(Board::COMMAND_STEERING_NR);
	}
}
void PsesUcBoard::setMotor(const int level){
	if(level > 20 || level <-20){
		throw UcBoardException(Board::COMMAND_MOTOR_OOB);
	}
	std::stringstream valueStream;
	if(level == 0){
		valueStream << "F " << 0; // was -500 -> active braking, caused a malfunction where driving backwards was no longer possible
	}else if(level>0){
		valueStream << "F " << 50*level;

	}else{
		valueStream << "B " << -25*level;
	}
	std::string value = valueStream.str();
	std::string command = "!DRV "+value;
	std::string answer;
	ros::Time start = ros::Time::now();
	do{
		sendRequest(command, answer);
		
	}while(answer.find(value)==-1 && (ros::Time::now()-start).toSec()<=0.1);
	if(answer.find(value)==-1){
		throw UcBoardException(Board::COMMAND_MOTOR_NR);
	}
}

void PsesUcBoard::activateKinect(){
	std::string command = "!VOUT ON";
	std::string answer;
	std::string value = ":ON";

	ros::Time start = ros::Time::now();
	do{
		sendRequest(command, answer);
	}while(answer.find(value)==-1 && (ros::Time::now()-start).toSec()<=0.1);
	if(answer.find(value)==-1){
		throw UcBoardException(Board::REQUEST_KINECT_ON);
	}

}

void PsesUcBoard::deactivateKinect(){
	std::string command = "!VOUT OFF";
	std::string answer;
	std::string value = ":OFF";

	ros::Time start = ros::Time::now();
	do{
		sendRequest(command, answer);
	}while(answer.find(value)==-1 && (ros::Time::now()-start).toSec()<=0.1);
	if(answer.find(value)==-1){
		throw UcBoardException(Board::REQUEST_KINECT_ON);
	}

}

void PsesUcBoard::getBoardError(std::string& msg){
	readInputBuffer();
	errorStack->pop(msg);
}
void PsesUcBoard::getBoardMessage(std::string& msg){
	readInputBuffer();
	displayStack->pop(msg);
}

bool PsesUcBoard::boardSensorValues(){
	readInputBuffer();
	return !sensorGroupStack->isEmpty();
}

bool PsesUcBoard::boardErrors(){
	readInputBuffer();
	return !errorStack->isEmpty();
}

bool PsesUcBoard::boardMessages(){
	readInputBuffer();
	return !displayStack->isEmpty();
}

void PsesUcBoard::queryCarID(){
	std::string command ="?ID";
	std::string answer;
	ros::Time start = ros::Time::now();
	do{
		sendRequest(command, answer);
		try{
			answer = answer.substr(1, answer.size()-1);
			carID = std::stoi(answer);
		}catch(std::exception& e){
			carID = -1;
		}
	}while(carID<0 && (ros::Time::now()-start).toSec()<=0.1);
	if(carID==-1){
		throw UcBoardException(Board::REQUEST_NO_ID);
	}

}
void PsesUcBoard::connect(const unsigned int serialTimeout){
	if(!connected){
		try{
		serialConnection.setPort("/dev/" + deviceName);
		serialConnection.setBaudrate(baudRate);
		serial::Timeout timeout = serial::Timeout::simpleTimeout(serialTimeout);
		serialConnection.setTimeout(timeout);
		serialConnection.open();
		}catch(serial::IOException& e){
			throw UcBoardException(Board::CONNECTING_FAILED);
		}
		if(serialConnection.isOpen()){
			connected = true;
		}else{
			throw UcBoardException(Board::CONNECTION_NOT_ESTABLISHED);
		}
	}else{
		throw UcBoardException(Board::CONNECTION_ALREADY_ESTABLISHED);
	}
}

void PsesUcBoard::sendRequest(const std::string& req, std::string& answer){
	if(!connected){
		throw UcBoardException(Board::CONNECTION_NOT_ESTABLISHED);
	}
	send(req);
	ros::Time start = ros::Time::now();
	do{
		readInputBuffer();
		responseStack->pop(answer);

	}while(answer.size()==0 && (ros::Time::now()-start).toSec()<=0.05);

	if(answer.size()!=0){
		answer = answer.substr(1,answer.size()-2);
	}
}

void PsesUcBoard::readInputBuffer(){
	if(!connected){
		throw UcBoardException(Board::CONNECTION_NOT_ESTABLISHED);
	}
	std::string input;
		receive(input);
		if(input.size()==0){
			input="";
			return;
		}
		if(input.find("\x03")==-1){
			input="";
			return;
		}
		if(input.find("##")!=-1 && input.find(":")!=-1){
			sensorGroupStack->push(input);
			input="";
			return;
		}
		if(input.find(":")!=-1 && input.find("##")==-1){
			responseStack->push(input);
			input="";
			return;
		}
		if(input.find("'")!=-1 && input.find("ERR")!=-1){
			errorStack->push(input);
			input="";
			return;
		}
		if(input.find("'")!=-1 && input.find("ERR")==-1){
			displayStack->push(input);
			input="";
			return;
		}

}

void PsesUcBoard::send(const std::string& msg) {
	if(!connected){
		throw UcBoardException(Board::CONNECTION_NOT_ESTABLISHED);
	}
	std::string outMsg = msg+"\n";
	if(!serialConnection.write(outMsg) && msg.size()>0){
		throw UcBoardException(Board::TRANSMISSION_FAILED);
	}
}

void PsesUcBoard::receive(std::string& msg) {
	if(!connected){
		throw UcBoardException(Board::CONNECTION_NOT_ESTABLISHED);
	}
	if(serialConnection.available()){
		serialConnection.readline(msg, 65536, "\x03");
	}else{
		msg = "";
	}

}

void PsesUcBoard::reset(){
	if(!connected){
		throw UcBoardException(Board::CONNECTION_NOT_ESTABLISHED);
	}
	std::string command = "!RESET NOW";
	send(command);
}

void PsesUcBoard::deactivateUCBoard(){
	if(!connected){
		throw UcBoardException(Board::CONNECTION_NOT_ESTABLISHED);
	}
	setSteering(0);
	setMotor(0);
	stopSensors();
	reset();
	}

	void PsesUcBoard::setSensorGroup(const Board::SensorGroup& sensors, const int numOfGroup, const std::string& parameter){
		if(sensors.size()==0){
			return;
		}
		std::string answer;
		std::string value = ":ok";
		std::stringstream command;
		command<<"!DAQ GRP "<<numOfGroup<<" "<<parameter;
		for(auto current : sensors){
			command<<" "<<Board::sensorTable[current];
		}

		ros::Time start = ros::Time::now();
		do{
			sendRequest(command.str(), answer);
		}while(answer.find(value)==-1 && (ros::Time::now()-start).toSec()<=0.1);
		if(answer.find(value)==-1){
			throw UcBoardException(Board::REQUEST_NO_GROUP);
		}

	}

	void PsesUcBoard::startSensors(){
		std::string command = "!DAQ START";
		std::string answer;
		std::string value = ":started";

		ros::Time start = ros::Time::now();
		do{
			sendRequest(command, answer);
		}while(answer.find(value)==-1 && (ros::Time::now()-start).toSec()<=0.1);
		if(answer.find(value)==-1){
			throw UcBoardException(Board::REQUEST_NO_START);
		}

	}

	void PsesUcBoard::stopSensors(){
		std::string command = "!DAQ STOP";
		std::string answer;
		std::string value = ":stopped";

		ros::Time start = ros::Time::now();
		do{
			sendRequest(command, answer);
		}while(answer.find(value)==-1 && (ros::Time::now()-start).toSec()<=0.1);
		if(answer.find(value)==-1){
			throw UcBoardException(Board::REQUEST_NO_STOP);
		}
	}

  void PsesUcBoard::getSensorData(pses_basis::SensorData& data){
		readInputBuffer();
		std::string rawData;
		sensorGroupStack->pop(rawData);
		if(rawData.size()==0){
			return;
		}
		//string must identify as vaild sensor group
		int start = rawData.find("##");
		int end = rawData.find("\x03");
		if(start<0 || end<0){
			throw UcBoardException(Board::SENSOR_PARSER_INVALID);
		}
		//get group ID
		int groupID = 0;
		int idBegin = start+2;
		int idLength = rawData.find(":")-idBegin;
		try{
			groupID = std::stoi(rawData.substr(idBegin, idLength));
		}catch(std::exception& e){
			throw UcBoardException(Board::SENSOR_ID_INVALID);
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
					if(rawData.find("[") != std::string::npos && rawData.find("]") != std::string::npos){
						sensorValue = std::numeric_limits<int>::quiet_NaN();
					}else{
						sensorValue = std::stoi(rawData);
					}
					assignSensorValue(sensorMessage, sensorValue, sensorGroups[groupID-1][sensorCount]);
				}catch(std::exception& e){
					throw UcBoardException(Board::SENSOR_PARSER_INVALID);
				}
			}else{
				try{
					std::string current = rawData.substr(0, nextSensor);
					if(current.find("[") != std::string::npos && current.find("]") != std::string::npos){
						sensorValue = std::numeric_limits<int>::quiet_NaN();
					}else{
						sensorValue = std::stoi(current);
					}
					assignSensorValue(sensorMessage, sensorValue, sensorGroups[groupID-1][sensorCount]);
					sensorCount++;
					start = nextSensor+3;
					substrLength = rawData.size();
					rawData = rawData.substr(start, substrLength);
				}catch(std::exception& e){
					throw UcBoardException(Board::SENSOR_PARSER_INVALID);
				}
			}
		}while(nextSensor>=0);
		data = sensorMessage;
	}

	void PsesUcBoard::assignSensorValue(pses_basis::SensorData& data, const int value , const Board::SensorObject& sensor){
		switch(sensor){
			case Board::accelerometerX :
				data.accelerometer_x = value*8.0/std::pow(2,16)*9.81;
				break;
			case Board::accelerometerY :
				data.accelerometer_y = value*8.0/std::pow(2,16)*9.81;
				break;
			case Board::accelerometerZ :
				data.accelerometer_z = value*8.0/std::pow(2,16)*9.81;
				break;
			case Board::gyroscopeX :
                data.angular_velocity_x = Board::degToRad(value*1000.0/std::pow(2,16));
				break;
			case Board::gyroscopeY :
                data.angular_velocity_y = Board::degToRad(value*1000.0/std::pow(2,16));
				break;
			case Board::gyroscopeZ :
                data.angular_velocity_z = Board::degToRad(value*1000.0/std::pow(2,16));
				break;
			case Board::rangeSensorLeft :
				data.range_sensor_left = value/10000.0;
				break;
			case Board::rangeSensorFront :
				data.range_sensor_front = value/10000.0;
				break;
			case Board::rangeSensorRight :
				data.range_sensor_right = value/10000.0;
				break;
			case Board::hallSensorDT :
				data.hall_sensor_dt = value/10000.0;
				break;
			case Board::hallSensorDTFull :
				data.hall_sensor_dt_full = value/1000.0;
				break;
			case Board::hallSensorCount :
				data.hall_sensor_count = value;
				break;
			case Board::batteryVoltageSystem :
				data.system_battery_voltage = value/1000.0;
				break;
			case Board::batteryVoltageMotor :
				data.motor_battery_voltage = value/1000.0;
				break;

		}
}
