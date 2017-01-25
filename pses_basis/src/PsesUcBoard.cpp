#include <pses_basis/PsesUcBoard.h>

PsesUcBoard::PsesUcBoard(const unsigned int baudRate, const std::string deviceName) : baudRate(baudRate), deviceName(deviceName){
								connected = false;
								errorStack = new InputStack(30);
								responseStack = new InputStack(30);
								displayStack = new InputStack(30);
								carID = -1;
								firmwareVersion = "no_info";
								motorLevel = -99999;
								steeringLevel = -99999;
								kinectOn = true;
								usOn = false;
}
PsesUcBoard::~PsesUcBoard() {
								delete errorStack;
								delete responseStack;
								delete displayStack;
}
void PsesUcBoard::initUcBoard(const unsigned int serialTimeout){
								connect(serialTimeout);
								setSteering(0);
								setMotor(0);
								queryCarID();
								queryFirmwareVersion();
								//activate us sensors
								activateUS();
								//generate sensor groups
								Board::SensorGroup sg;
								std::string params;
								//US-Sensor Group
								sg = {Board::rangeSensorLeft, Board::rangeSensorFront, Board::rangeSensorRight};
								params = "~ALL=5";
								sensorGroups.push_back(sg);
								groupStacks.push_back(InputStack(20));
								setSensorGroup(sg, 1, params);
								//Accel-Sensor Group
								sg = {Board::accelerometerX, Board::accelerometerY, Board::accelerometerZ};
								params = "~TS=10 ~AVG";
								sensorGroups.push_back(sg);
								groupStacks.push_back(InputStack(20));
								setSensorGroup(sg, 2, params);
								//Gyro-Sensor Group
								sg = {Board::gyroscopeX, Board::gyroscopeY, Board::gyroscopeZ};
								params = "~TS=10 ~AVG";
								sensorGroups.push_back(sg);
								groupStacks.push_back(InputStack(20));
								setSensorGroup(sg, 3, params);
								setGyroSensitivity(2000);
								//Hall-Sensor Group
								sg = {Board::hallSensorDT, Board::hallSensorDTFull, Board::hallSensorCount};
								params = "~ALL=5";
								sensorGroups.push_back(sg);
								groupStacks.push_back(InputStack(20));
								setSensorGroup(sg, 4, params);
								//Misc Group
								sg = {Board::batteryVoltageSystem, Board::batteryVoltageMotor};
								params = "~ALL=500";
								sensorGroups.push_back(sg);
								groupStacks.push_back(InputStack(20));
								setSensorGroup(sg, 5, params);
								//Magnetometer Group
								sg = {Board::magnetometerX, Board::magnetometerY, Board::magnetometerZ};
								params = "";
								sensorGroups.push_back(sg);
								groupStacks.push_back(InputStack(20));
								setSensorGroup(sg, 6, params);

								startSensors();
}
const int PsesUcBoard::getId() const {
								return carID;
}
const std::string& PsesUcBoard::getFirmwareVersion() const {
								return firmwareVersion;
}
void PsesUcBoard::setSteering(const int level){
								if(level > 50 || level <-50) {
																throw UcBoardException(Board::COMMAND_STEERING_OOB);
								}

								if(level == steeringLevel) return;

								std::stringstream valueStream;
								std::stringstream checkStream;
								valueStream << level*(-20);
								std::string value = valueStream.str();
								std::string command = "!STEER " + value;
								std::string answer;
								ros::Time start = ros::Time::now();
								do {
																sendRequest(command, answer);
								} while(answer.find(value)==-1 && (ros::Time::now()-start).toSec()<=0.1);

								if(answer.find(value)==-1) {
																throw UcBoardException(Board::COMMAND_STEERING_NR);
								}

								steeringLevel = level;
}
void PsesUcBoard::setMotor(const int level){
								if(level > 20 || level <-20) {
																throw UcBoardException(Board::COMMAND_MOTOR_OOB);
								}

								if(level == motorLevel) return;

								std::stringstream valueStream;
								if(level == 0) {
																valueStream << "F " << 0; // was -500 -> active braking, caused a malfunction where driving backwards was no longer possible
								}else if(level>0) {
																valueStream << "F " << 50*level;

								}else{
																valueStream << "B " << -25*level;
								}
								std::string value = valueStream.str();
								std::string command = "!DRV "+value;
								std::string answer;
								ros::Time start = ros::Time::now();
								do {
																sendRequest(command, answer);

								} while(answer.find(value)==-1 && (ros::Time::now()-start).toSec()<=0.1);
								if(answer.find(value)==-1) {
																throw UcBoardException(Board::COMMAND_MOTOR_NR);
								}

								motorLevel = level;
}

void PsesUcBoard::activateUS(){
								if(usOn) return;
								std::string command = "!US ON";
								std::string answer;
								std::string value = ":ON";

								ros::Time start = ros::Time::now();
								do {
																sendRequest(command, answer);
								} while(answer.find(value)==-1 && (ros::Time::now()-start).toSec()<=0.1);
								if(answer.find(value)==-1) {
																throw UcBoardException(Board::REQUEST_US_ON);
								}
								usOn = true;
}
void PsesUcBoard::deactivateUS(){
								if(!usOn) return;
								std::string command = "!US OFF";
								std::string answer;
								std::string value = ":OFF";

								ros::Time start = ros::Time::now();
								do {
																sendRequest(command, answer);
								} while(answer.find(value)==-1 && (ros::Time::now()-start).toSec()<=0.1);
								if(answer.find(value)==-1) {
																throw UcBoardException(Board::REQUEST_US_OFF);
								}
								usOn = false;
}

void PsesUcBoard::activateKinect(){
								if(kinectOn) return;
								std::string command = "!VOUT ON";
								std::string answer;
								std::string value = ":ON";

								ros::Time start = ros::Time::now();
								do {
																sendRequest(command, answer);
								} while(answer.find(value)==-1 && (ros::Time::now()-start).toSec()<=0.1);
								if(answer.find(value)==-1) {
																throw UcBoardException(Board::REQUEST_KINECT_ON);
								}
								kinectOn = true;

}

void PsesUcBoard::deactivateKinect(){
								if(!kinectOn) return;
								std::string command = "!VOUT OFF";
								std::string answer;
								std::string value = ":OFF";

								ros::Time start = ros::Time::now();
								do {
																sendRequest(command, answer);
								} while(answer.find(value)==-1 && (ros::Time::now()-start).toSec()<=0.1);
								if(answer.find(value)==-1) {
																throw UcBoardException(Board::REQUEST_KINECT_OFF);
								}
								kinectOn = false;

}
void PsesUcBoard::setGyroSensitivity(unsigned int sensitivity){
								std::string command = "!IMU OPT ~GRANGE="+std::to_string(sensitivity);
								std::string answer;
								std::string value = "~GRANGE="+std::to_string(sensitivity);

								ros::Time start = ros::Time::now();
								do {
																sendRequest(command, answer);
								} while(answer.find(value)==-1 && (ros::Time::now()-start).toSec()<=0.1);
								if(answer.find(value)==-1) {
																throw UcBoardException(Board::REQUEST_SET_GYRO);
								}
}
void PsesUcBoard::getBoardError(std::string& msg){
								readInputBuffer();
								errorStack->pop(msg);
								int strayBreak = msg.find("\n");
								int strayEOT = msg.find("\x03");
								if(strayBreak!=-1) msg.at(strayBreak)='/';
								if(strayEOT!=-1) msg.at(strayEOT)='/';
}
void PsesUcBoard::getBoardMessage(std::string& msg){
								readInputBuffer();
								displayStack->pop(msg);
								int strayBreak = msg.find("\n");
								int strayEOT = msg.find("\x03");
								if(strayBreak!=-1) msg.at(strayBreak)='/';
								if(strayEOT!=-1) msg.at(strayEOT)='/';
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
								do {
																sendRequest(command, answer);
																try{
																								answer = answer.substr(1, answer.size()-1);
																								carID = std::stoi(answer);
																}catch(std::exception& e) {
																								carID = -1;
																}
								} while(carID<0 && (ros::Time::now()-start).toSec()<=0.1);
								if(carID==-1) {
																throw UcBoardException(Board::REQUEST_NO_ID);
								}

}
void PsesUcBoard::queryFirmwareVersion(){
								std::string command ="?VER";
								std::string answer;
								ros::Time start = ros::Time::now();
								do {
																sendRequest(command, answer);
																answer = answer.substr(1, answer.size()-1);
								} while(answer.size()<=0 && (ros::Time::now()-start).toSec()<=0.1);
								if(answer.size()<=0) {
																throw UcBoardException(Board::REQUEST_NO_FWV);
								}
								int strayBreak = answer.find("\n");
								int strayEOT = answer.find("\x03");
								if(strayBreak!=-1) answer.at(strayBreak)=' ';
								if(strayEOT!=-1) answer.at(strayEOT)=' ';
								firmwareVersion = answer;
}
void PsesUcBoard::connect(const unsigned int serialTimeout){
								if(!connected) {
																try{
																								serialConnection.setPort("/dev/" + deviceName);
																								serialConnection.setBaudrate(baudRate);
																								serial::Timeout timeout = serial::Timeout::simpleTimeout(serialTimeout);
																								serialConnection.setTimeout(timeout);
																								serialConnection.open();
																}catch(serial::IOException& e) {
																								throw UcBoardException(Board::CONNECTING_FAILED);
																}
																if(serialConnection.isOpen()) {
																								connected = true;
																}else{
																								throw UcBoardException(Board::CONNECTION_NOT_ESTABLISHED);
																}
								}else{
																throw UcBoardException(Board::CONNECTION_ALREADY_ESTABLISHED);
								}
}

void PsesUcBoard::sendRequest(const std::string& req, std::string& answer){
								if(!connected) {
																throw UcBoardException(Board::CONNECTION_NOT_ESTABLISHED);
								}
								send(req);
								ros::Time start = ros::Time::now();
								do {
																readInputBuffer();
																responseStack->pop(answer);

								} while(answer.size()==0 && (ros::Time::now()-start).toSec()<=0.05);

								if(answer.size()!=0) {
																answer = answer.substr(1,answer.size()-2);
								}
}

void PsesUcBoard::readInputBuffer(){
								if(!connected) {
																throw UcBoardException(Board::CONNECTION_NOT_ESTABLISHED);
								}
								std::string input;
								receive(input);
								if(input.size()==0) {
																input="";
																return;
								}
								if(input.find("\x03")==-1) {
																input="";
																return;
								}
								if(input.find("##")!=-1 && input.find(":")!=-1) {
																putGroupInStack(input);
																input="";
																return;
								}
								if(input.find(":")!=-1 && input.find("##")==-1) {
																responseStack->push(input);
																input="";
																return;
								}
								if(input.find("'")!=-1 && input.find("ERR")!=-1) {
																errorStack->push(input);
																input="";
																return;
								}
								if(input.find("'")!=-1 && input.find("ERR")==-1) {
																displayStack->push(input);
																input="";
																return;
								}

}



void PsesUcBoard::send(const std::string& msg) {
								if(!connected) {
																throw UcBoardException(Board::CONNECTION_NOT_ESTABLISHED);
								}
								std::string outMsg = msg+"\n";
								if(!serialConnection.write(outMsg) && msg.size()>0) {
																throw UcBoardException(Board::TRANSMISSION_FAILED);
								}
}

void PsesUcBoard::receive(std::string& msg) {
								if(!connected) {
																throw UcBoardException(Board::CONNECTION_NOT_ESTABLISHED);
								}
								if(serialConnection.available()) {
																serialConnection.readline(msg, 65536, "\x03");
								}else{
																msg = "";
								}

}

void PsesUcBoard::reset(){
								if(!connected) {
																throw UcBoardException(Board::CONNECTION_NOT_ESTABLISHED);
								}
								std::string command = "!RESET NOW";
								send(command);
}

void PsesUcBoard::deactivateUCBoard(){
								if(!connected) {
																throw UcBoardException(Board::CONNECTION_NOT_ESTABLISHED);
								}
								setSteering(0);
								setMotor(0);
								stopSensors();
								if(usOn) deactivateUS();
								reset();
								if(connected) {
																serialConnection.close();
								}
}

void PsesUcBoard::setSensorGroup(const Board::SensorGroup& sensors, const int numOfGroup, const std::string& parameter){
								if(sensors.size()==0) {
																return;
								}
								std::string answer;
								std::string value = ":ok";
								std::stringstream command;
								command<<"!DAQ GRP "<<numOfGroup<<" "<<parameter;
								for(auto current : sensors) {
																command<<" "<<Board::sensorTable[current];
								}

								ros::Time start = ros::Time::now();
								do {
																sendRequest(command.str(), answer);
								} while(answer.find(value)==-1 && (ros::Time::now()-start).toSec()<=0.1);
								if(answer.find(value)==-1) {
																throw UcBoardException(Board::REQUEST_NO_GROUP);
								}

}

void PsesUcBoard::startSensors(){
								std::string command = "!DAQ START";
								std::string answer;
								std::string value = ":started";

								ros::Time start = ros::Time::now();
								do {
																sendRequest(command, answer);
								} while(answer.find(value)==-1 && (ros::Time::now()-start).toSec()<=0.1);
								if(answer.find(value)==-1) {
																throw UcBoardException(Board::REQUEST_NO_START);
								}

}

void PsesUcBoard::stopSensors(){
								std::string command = "!DAQ STOP";
								std::string answer;
								std::string value = ":stopped";

								ros::Time start = ros::Time::now();
								do {
																sendRequest(command, answer);
								} while(answer.find(value)==-1 && (ros::Time::now()-start).toSec()<=0.1);
								if(answer.find(value)==-1) {
																throw UcBoardException(Board::REQUEST_NO_STOP);
								}
}

void PsesUcBoard::putGroupInStack(const std::string& groupMsg){
								if(groupMsg.size()==0) {
																return;
								}
								//string must identify as vaild sensor group
								int start = groupMsg.find("##");
								int end = groupMsg.find("\x03");
								if(start<0 || end<0) {
																throw UcBoardException(Board::SENSOR_PARSER_INVALID);
								}
								//get group ID
								int groupID = 0;
								int idBegin = start+2;
								int idLength = groupMsg.find(":")-idBegin;
								try{
																groupID = std::stoi(groupMsg.substr(idBegin, idLength));
								}catch(std::exception& e) {
																throw UcBoardException(Board::SENSOR_ID_INVALID);
								}
								//remove preamble and trails
								start = idBegin+idLength+1;
								int substrLength = end-start;
								std::string out = groupMsg.substr(idBegin+idLength+1, substrLength);
								groupStacks[groupID-1].push(out);

}

void PsesUcBoard::getSensorData(pses_basis::SensorData& data){
								readInputBuffer();

								//set message meta data
								sensorMessage.header.seq++;
								sensorMessage.header.stamp = ros::Time::now();
								sensorMessage.header.frame_id = "ucBoard";
								//reset wheel measurements
								sensorMessage.hall_sensor_dt = std::numeric_limits<float>::quiet_NaN();
								sensorMessage.hall_sensor_dt_full = std::numeric_limits<float>::quiet_NaN();

								for(int groupID = 1; groupID<=groupStacks.size(); groupID++) {
																InputStack& group = groupStacks[groupID-1];
																// pop group data from stack
																// if stack empty -> skip group
																std::string rawData;
																group.pop(rawData);
																if(rawData.size()==0) continue;
																//parse sensor values
																int sensorCount = 0;
																int nextSensor = -1;
																int sensorValue = 0;
																do {
																								nextSensor = rawData.find(" | ");
																								if(nextSensor<0) {
																																try{
																																								if(rawData.find("[") != std::string::npos && rawData.find("]") != std::string::npos) {
																																																sensorValue = std::numeric_limits<int>::quiet_NaN();
																																								}else{
																																																sensorValue = std::stoi(rawData);
																																								}
																																								assignSensorValue(sensorMessage, sensorValue, sensorGroups[groupID-1][sensorCount]);
																																}catch(std::exception& e) {
																																								throw UcBoardException(Board::SENSOR_PARSER_INVALID);
																																}
																								}else{
																																try{
																																								std::string current = rawData.substr(0, nextSensor);
																																								if(current.find("[") != std::string::npos && current.find("]") != std::string::npos) {
																																																sensorValue = std::numeric_limits<int>::quiet_NaN();
																																								}else{
																																																sensorValue = std::stoi(current);
																																								}
																																								assignSensorValue(sensorMessage, sensorValue, sensorGroups[groupID-1][sensorCount]);
																																								sensorCount++;
																																								rawData = rawData.substr(nextSensor+3, rawData.size());
																																}catch(std::exception& e) {
																																								throw UcBoardException(Board::SENSOR_PARSER_INVALID);
																																}
																								}
																} while(nextSensor>=0);

								}
								data = sensorMessage;
}

void PsesUcBoard::assignSensorValue(pses_basis::SensorData& data, const int value, const Board::SensorObject& sensor){
								switch(sensor) {
								case Board::accelerometerX:
																data.accelerometer_x = value*8.0/std::pow(2,16)*9.81;
																break;
								case Board::accelerometerY:
																data.accelerometer_y = value*8.0/std::pow(2,16)*9.81;
																break;
								case Board::accelerometerZ:
																data.accelerometer_z = value*8.0/std::pow(2,16)*9.81;
																break;
								case Board::gyroscopeX:
																data.angular_velocity_x = Board::degToRad(value/16.4);
																break;
								case Board::gyroscopeY:
																data.angular_velocity_y = Board::degToRad(value/16.4);
																break;
								case Board::gyroscopeZ:
																data.angular_velocity_z = Board::degToRad(value/16.4);
																break;
								case Board::rangeSensorLeft:
																data.range_sensor_left = value/10000.0;
																break;
								case Board::rangeSensorFront:
																data.range_sensor_front = value/10000.0;
																break;
								case Board::rangeSensorRight:
																data.range_sensor_right = value/10000.0;
																break;
								case Board::hallSensorDT:
																data.hall_sensor_dt = value/10000.0;
																break;
								case Board::hallSensorDTFull:
																data.hall_sensor_dt_full = value/1000.0;
																break;
								case Board::hallSensorCount:
																data.hall_sensor_count = value;
																break;
								case Board::batteryVoltageSystem:
																data.system_battery_voltage = value/1000.0;
																break;
								case Board::batteryVoltageMotor:
																data.motor_battery_voltage = value/1000.0;
																break;
								case Board::magnetometerX:
																data.magnetometer_x = value*0.00000015;
																break;
								case Board::magnetometerY:
																data.magnetometer_y = value*0.00000015;
																break;
								case Board::magnetometerZ:
																data.magnetometer_z = value*0.00000015;
																break;
								}
}
