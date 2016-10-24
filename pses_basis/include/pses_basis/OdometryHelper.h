#ifndef OdometryHelper_H
#define OdometryHelper_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <pses_basis/SensorData.h>
#include <pses_basis/Command.h>
#include <pses_basis/ForwardKinematics.h>
#include <math.h>
#include <sensor_msgs/Imu.h>

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
        prevDirection = 0;
		oldTimeStamp = ros::Time::now();
		odometric.setK(0.25); // set distance between front axis and back axis (in meters)
	}
	inline void updateSensorData(const pses_basis::SensorData& sensorData) {
		this->sensorData = sensorData;
		calcDt(sensorData.header.stamp, oldTimeStamp);
		oldTimeStamp = sensorData.header.stamp;
		calcSpeed();
		calcDeltaDistance();
		calcDrivenDistance();
		calcPosition();
	}
	inline void updateIMUData(sensor_msgs::Imu& imu){
		this->imu = imu;
		tf::Quaternion q;
		tf::quaternionMsgToTF(imu.orientation, q);
		tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	}
	inline void updateCommand(const pses_basis::Command& cmd) {
		command = cmd;
        prevDirection = drivingDirection;
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
	inline void getQuaternion(geometry_msgs::Quaternion& quat) {
		quat = imu.orientation;
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
    int prevDirection;
	int drivingDirection; // -1 = backwards; 0 = stop; 1 = forwards
	ros::Time oldTimeStamp;
	geometry_msgs::Point position;
	ForwardKinematics odometric; // object needed for odometric calculations
	sensor_msgs::Imu imu;
	pses_basis::SensorData sensorData;
	pses_basis::Command command;
	inline void calcDt(const ros::Time& currentTimeStamp, const ros::Time& oldTimeStamp) {
		dt = (currentTimeStamp - oldTimeStamp).toSec();
	}
	inline void calcSpeed() {
        if(!isnan(sensorData.hall_sensor_dt)){
            speed =  drivingDirection * DRIVEN_DISTANCE_PER_TICK / sensorData.hall_sensor_dt;
        }else{
            if(prevDirection!=drivingDirection) speed = 0;
            else if(drivingDirection == 0) speed =  0;
        }
	}
	inline void calcDeltaDistance() { 
		//return speed*dt;
		deltaDistance = isnan(sensorData.hall_sensor_dt)?0.0 : drivingDirection*DRIVEN_DISTANCE_PER_TICK;
	}
	inline void calcDrivenDistance() {
		drivenDistance = drivenDistance + deltaDistance;
	}
	inline void calcPosition() {
		std::vector<double> pos = odometric.getUpdateWithGyro(yaw, deltaDistance);
		position.x = pos.at(1);
		position.y = -pos.at(0);
		position.z = 0.0;
	}
};

#endif
