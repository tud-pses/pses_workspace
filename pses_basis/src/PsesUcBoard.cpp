#include <pses_basis/PsesUcBoard.h>

PsesUcBoard::PsesUcBoard(const unsigned int baudRate, const std::string deviceName) : baudRate(baudRate), deviceName(deviceName){
	connected = false;
	errorStack = new InputStack(100);
	responseStack = new InputStack(100);
	sensorGroupStack = new InputStack(100);
	displayStack = new InputStack(100);
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
        //ROS_INFO_STREAM("<<Command:"<<command<<">>"<<"<<Query:"<<check<<">>"<<"<<Answer:"<<answer<<">>");
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
		valueStream << "F " << -500;
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
		//ROS_INFO_STREAM("<<Query:"<<value<<">>"<<"<<Answer:"<<answer<<">>");
	}while(answer.find(value)==-1 && (ros::Time::now()-start).toSec()<=0.1);
	if(answer.find(value)==-1){
		throw UcBoardException(Board::COMMAND_MOTOR_NR);
	}
}
void PsesUcBoard::emptyAllStacks(){
	std::string out;
	while(!errorStack->isEmpty()){
		errorStack->pop(out);
		ROS_INFO_STREAM("<<Error: "<<out<<" >>");
		out="";
	}
	while(!responseStack->isEmpty()){
		responseStack->pop(out);
		ROS_INFO_STREAM("<<Response: "<<out<<" >>");
		out="";
	}
	while(!sensorGroupStack->isEmpty()){
		sensorGroupStack->pop(out);
		ROS_INFO_STREAM("<<Group: "<<out<<" >>");
		out="";
	}
	while(!displayStack->isEmpty()){
		displayStack->pop(out);
		ROS_INFO_STREAM("<<Display: "<<out<<" >>");
		out="";
	}
}
void PsesUcBoard::queryCarID(){
	std::string command ="?ID";
	std::string answer;
	ros::Time start = ros::Time::now();
	do{
		sendRequest(command, answer);
		try{
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
	//readInputBuffer();
	send(req);
	ros::Time start = ros::Time::now();
	do{
		readInputBuffer();
		responseStack->pop(answer);
		//ros::Duration(0.001).sleep();

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
			ROS_INFO_STREAM("<<RAW-zero: "<<input<<" size: "<< input.size() <<" >>");
			input="";
			//ros::Duration(0.001).sleep();
			return;
		}
		if(input.find("\x03")==-1){
			ROS_INFO_STREAM("<<RAW-broken: "<<input<<" size: "<< input.size() <<" >>");
			input="";
			//ros::Duration(0.001).sleep();
			return;
		}
		if(input.find("##")!=-1 && input.find(":")!=-1){
			ROS_INFO_STREAM("<<group: "<<input<<" size: "<< input.size() <<" >>");
			sensorGroupStack->push(input);
			input="";
			//ros::Duration(0.001).sleep();
			return;
		}
		if(input.find(":")!=-1 && input.find("##")==-1){
			ROS_INFO_STREAM("<<response: "<<input<<" size: "<< input.size() <<" >>");
			responseStack->push(input);
			input="";
			//ros::Duration(0.001).sleep();
			return;
		}
		if(input.find("'")!=-1 && input.find("ERR")!=-1){
			ROS_INFO_STREAM("<<error: "<<input<<" size: "<< input.size() <<" >>");
			errorStack->push(input);
			input="";
			//ros::Duration(0.001).sleep();
			return;
		}
		if(input.find("'")!=-1 && input.find("ERR")==-1){
			ROS_INFO_STREAM("<<display: "<<input<<" size: "<< input.size() <<" >>");
			displayStack->push(input);
			input="";
			//ros::Duration(0.001).sleep();
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

void PsesUcBoard::deactivateUCBoard(){
	if(!connected){
		throw UcBoardException(Board::CONNECTION_NOT_ESTABLISHED);
	}
	setSteering(0);
	setMotor(0);
	//send stop daq
	//send reset
	}
