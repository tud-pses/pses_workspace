#ifndef OdometryHelper_H
#define OdometryHelper_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <pses_basis/SensorData.h>
#include <pses_basis/Command.h>
#include <pses_basis/ForwardKinematics.h>

#define PI  3.1415926535897932384626433832795
#define WHEEL_RADIUS 0.032
#define RAD_PER_TICK 0.7853981634 // 2*PI/8
#define DRIVEN_DISTANCE_PER_TICK 0.0251327412 // RAD_PER_TICK * WHEEL_RADIUS 

class OdometryHelper{
public:
	OdometryHelper() {
		yaw = 0.0;
		dt = 0.0;
		drivenDistance = 0.0;
		speed = 0.0;
		drivingDirection = 0;
		oldTimeStamp = ros::Time::now();
		odometric.setK(0.25); // set distance between front axis and back axis (in meters)	
	}
	inline void updateSensorData(const pses_basis::SensorData& sensorData) {
		this->sensorData = &sensorData;
		dt = calcDt(sensorData.header.stamp, oldTimeStamp);
		oldTimeStamp = sensorData.header.stamp;
		speed = calcSpeed();
		yaw = calcYaw();
		position = calcPosition();
	}
	inline void updateCommand(const pses_basis::Command& cmd) {
		command = &cmd;
		if(command->motor_level < 0) drivingDirection = -1;
		else if(command->motor_level > 0) drivingDirection = 1;
		else drivingDirection = 0;
	}
	inline double getYaw() {
		return yaw;
	}
	inline float getSpeed() {
		return speed;
	}
	inline geometry_msgs::Point getPosition() {
		return position;
	}
	inline float getDrivenDistance() {
		drivenDistance = drivenDistance + calcDeltaDistance();
	}
	inline ~OdometryHelper() {
		delete sensorData;
		delete command;
	}
private:
	double yaw;
	double dt;
	float speed;
	float drivenDistance;
	int drivingDirection; // -1 = backwards; 0 = stop; 1 = forwards
	ros::Time oldTimeStamp;
	geometry_msgs::Point position;
	ForwardKinematics odometric; // object needed for odometric calculations
	const pses_basis::SensorData* sensorData;
	const pses_basis::Command* command;
	inline double calcDt(const ros::Time& currentTimeStamp, const ros::Time& oldTimeStamp) const{
		return (currentTimeStamp - oldTimeStamp).toSec();
	}
	inline float calcSpeed() {
		float hall_sensor_dt = sensorData->hall_sensor_dt;
		return drivingDirection * DRIVEN_DISTANCE_PER_TICK / hall_sensor_dt;
	}
	inline double calcYaw() {
		return yaw + degToRad(sensorData->angular_velocity_z*dt);
	}
	inline float calcDeltaDistance() { 
		return speed*dt; 
	}
	inline geometry_msgs::Point calcPosition() {
		std::vector<double> pos = odometric.getUpdateWithGyro(yaw, calcDeltaDistance());
		geometry_msgs::Point currentPosition;
		currentPosition.x = pos.at(1);
		currentPosition.y = -pos.at(0);
		currentPosition.z = 0.0;
		return currentPosition;
	}
	inline float degToRad(const float value) {
		return value*PI/180;
	}
};

#endif