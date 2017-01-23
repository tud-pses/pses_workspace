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
const std::string REQUEST_NO_FWV = "No valid answer on request for firmware Version.";
const std::string REQUEST_NO_GROUP = "Sensor group not set.";
const std::string REQUEST_NO_START = "Request to start groups not send.";
const std::string REQUEST_NO_STOP = "Request to stop groups not send.";
const std::string REQUEST_SET_GYRO = "Request to set Gyro-Parameters not send.";
const std::string REQUEST_KINECT_ON = "Request to activate Kinect not send.";
const std::string REQUEST_KINECT_OFF = "Request to deactivate Kinect not send.";
const std::string REQUEST_US_ON = "Request to activate US sensors not send.";
const std::string REQUEST_US_OFF = "Request to deactivate US sensors not send.";
const std::string SENSOR_PARSER_INVALID = "Invalid sensor group message.";
const std::string SENSOR_ID_INVALID = "Invalid sensor group ID.";

static std::vector<std::string> sensorTable = {"USL", "USF", "USR", "AX", "AY", "AZ", "GX", "GY", "GZ", "HALL_DT", "HALL_DT8", "HALL_CNT", "VSBAT", "VDBAT", "MX", "MY", "MZ"};

enum SensorObject {
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
								magnetometerX = 14,
								magnetometerY = 15,
								magnetometerZ = 16,
};

typedef std::vector<SensorObject> SensorGroup;
typedef std::vector<SensorGroup> SensorGroups;
typedef std::vector<InputStack> GroupMessages;

float degToRad(const float value) {
								return value*M_PI/180;
}
}

class PsesUcBoard {
public:
								PsesUcBoard(const unsigned int baudRate=921600, const std::string deviceName="ttyUSB0");
								~PsesUcBoard();
								void initUcBoard(const unsigned int serialTimeout=5);
								const int getId() const;
								const std::string& getFirmwareVersion() const;
								void setSteering(const int level);
								void setMotor(const int level);
								void activateUS();
								void deactivateUS();
								void activateKinect();
								void deactivateKinect();
								void getSensorData(pses_basis::SensorData& data);
								bool boardErrors();
								bool boardMessages();
								void getBoardError(std::string& msg);
								void getBoardMessage(std::string& msg);
								void deactivateUCBoard();

private:
								unsigned int baudRate;
								std::string deviceName;
								serial::Serial serialConnection;
								bool connected;
								InputStack* errorStack;
								InputStack* responseStack;
								InputStack* displayStack;
								int carID;
								std::string firmwareVersion;
								Board::SensorGroups sensorGroups;
								pses_basis::SensorData sensorMessage;
								Board::GroupMessages groupStacks;
								int motorLevel;
								int steeringLevel;
								bool kinectOn;
								bool usOn;

								void connect(const unsigned int serialTimeout);
								void send(const std::string& msg);
								void receive(std::string& msg);
								void reset();
								void readInputBuffer();
								void sendRequest(const std::string& req, std::string& answer);
								void queryCarID();
								void queryFirmwareVersion();
								void startSensors();
								void stopSensors();
								void setSensorGroup(const Board::SensorGroup& sensors, const int numOfGroup, const std::string& parameter);
								void setGyroSensitivity(unsigned int value);
								void putGroupInStack(const std::string& groupMsg);
								void assignSensorValue(pses_basis::SensorData& data, const int value, const Board::SensorObject& sensor);

};

#endif
