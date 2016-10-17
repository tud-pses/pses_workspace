#ifndef OdometryHelper_H
#define OdometryHelper_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <pses_basis/SensorData.h>
#include <pses_basis/Command.h>
#include <pses_basis/ForwardKinematics.h>
#include <math.h>

#define WHEEL_RADIUS 0.032
#define RAD_PER_TICK 0.7853981634 // 2*PI/8
#define DRIVEN_DISTANCE_PER_TICK 0.0251327412 // RAD_PER_TICK * WHEEL_RADIUS 

class OdometryHelper{
public:
	OdometryHelper() {
		yaw = 0.0;
		pitch = 0.0;
		roll = 0.0;
		dt = 0.0;
		drivenDistance = 0.0;
		speed = 0.0;
		drivingDirection = 0;
		oldTimeStamp = ros::Time::now();
		odometric.setK(0.25); // set distance between front axis and back axis (in meters)	
	}
	inline void updateSensorData(const pses_basis::SensorData& sensorData) {
		this->sensorData = sensorData;
		dt = calcDt(sensorData.header.stamp, oldTimeStamp);
		oldTimeStamp = sensorData.header.stamp;
		speed = calcSpeed();
		yaw = calcYaw();
		pitch = calcPitch();
		roll = calcRoll();
		deltaDistance = calcDeltaDistance();
		drivenDistance = calcDrivenDistance();
		position = calcPosition();
	}
	inline void updateCommand(const pses_basis::Command& cmd) {
		command = cmd;
		if(command.motor_level < 0) drivingDirection = -1;
		else if(command.motor_level > 0) drivingDirection = 1;
		else drivingDirection = 0;
	}
	inline double getYaw() {
		return yaw;
	}
	inline double getRoll() {
		return roll;
	}
	inline double getPitch() {
		return pitch;
	}
	inline float getSpeed() {
		return speed;
	}
	inline float getDrivenDistance() {
		return drivenDistance;
	}
	inline geometry_msgs::Point getPosition() {
		return position;
	}
private:
	double yaw;
	double roll;
	double pitch;
	double dt;
	float speed;
	float drivenDistance;
	float deltaDistance;
	int drivingDirection; // -1 = backwards; 0 = stop; 1 = forwards
	ros::Time oldTimeStamp;
	geometry_msgs::Point position;
	ForwardKinematics odometric; // object needed for odometric calculations
	pses_basis::SensorData sensorData;
	pses_basis::Command command;
	inline double calcDt(const ros::Time& currentTimeStamp, const ros::Time& oldTimeStamp) const{
		return (currentTimeStamp - oldTimeStamp).toSec();
	}
	inline float calcSpeed() {
		float hall_sensor_dt = sensorData.hall_sensor_dt;
		return isnan(hall_sensor_dt)? 0.0 : drivingDirection * DRIVEN_DISTANCE_PER_TICK / hall_sensor_dt;
	}
	inline double calcYaw() {
		return yaw + degToRad(sensorData.angular_velocity_z*dt);
	}
	inline double calcRoll() {
		return roll + degToRad(sensorData.angular_velocity_x*dt);
	}
	inline double calcPitch() {
		return pitch + degToRad(sensorData.angular_velocity_y*dt);
	}
	inline float calcDeltaDistance() { 
		return speed*dt; 
	}
	inline float calcDrivenDistance() {
		return drivenDistance + deltaDistance;
	}
	inline geometry_msgs::Point calcPosition() {
		std::vector<double> pos = odometric.getUpdateWithGyro(yaw, deltaDistance);
		geometry_msgs::Point currentPosition;
		currentPosition.x = pos.at(1);
		currentPosition.y = -pos.at(0);
		currentPosition.z = 0.0;
		return currentPosition;
	}
	inline float degToRad(const float value) {
		return value*M_PI/180;
	}
};

#endif