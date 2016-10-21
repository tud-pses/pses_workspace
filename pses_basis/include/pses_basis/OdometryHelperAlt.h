#ifndef OdometryHelperAlt_H
#define OdometryHelperAlt_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <pses_basis/SensorData.h>
#include <pses_basis/Command.h>
#include <pses_basis/ForwardKinematics.h>
#include <opencv2/opencv.hpp>

#define WHEEL_RADIUS 0.032
#define RAD_PER_TICK 0.7853981634 // 2*PI/8
#define DRIVEN_DISTANCE_PER_TICK 0.0251327412 // RAD_PER_TICK * WHEEL_RADIUS

class OdometryHelperAlt{
public:
	OdometryHelperAlt() {
		yaw = 0.0;
		pitch = 0.0;
		roll = 0.0;
		dt = 0.0;
		drivenDistance = 0.0;
		speed = 0.0;
		drivingDirection = 0;
		oldTimeStamp = ros::Time::now();
		odometric.setK(0.25); // set distance between front axis and back axis (in meters)
		gyroFilter = cv::KalmanFilter( 6, 3, 0 );
		setIdentity(gyroFilter.processNoiseCov, cv::Scalar::all(10));
		/*
		gyroFilter.processNoiseCov = (cv::Mat_<float>(6, 6) << 0.001, 0, 0, 0, 0, 0,
	                                                           0, 0.001, 0, 0, 0, 0,
	                                                           0, 0, 0.001, 0, 0, 0,
	                                                           0, 0, 0, 0.003, 0, 0,
	                                                           0, 0, 0, 0, 0.003, 0,
	                                                           0, 0, 0, 0, 0, 0.003);
	    */                                                       
		cv::setIdentity(gyroFilter.measurementMatrix);
		gyroFilter.measurementMatrix = (cv::Mat_<float>(3, 6) << 0, 0, 0, 1, 0, 0,
	                                                            0, 0, 0, 0, 1, 0,
	                                                            0, 0, 0, 0, 0, 1);
		setIdentity(gyroFilter.measurementNoiseCov, cv::Scalar::all(10));
	    setIdentity(gyroFilter.errorCovPost, cv::Scalar::all(1000));
		gyroFilter.transitionMatrix = (cv::Mat_<float>(6, 6) << 1, 0, 0, dt, 0, 0,
	                                                           0, 1, 0, 0, dt, 0,
	                                                           0, 0, 1, 0, 0, dt,
	                                                           0, 0, 0, 1, 0, 0,
	                                                           0, 0, 0, 0, 1, 0,
	                                                           0, 0, 0, 0, 0, 1);
		gyroFilter.statePost.at<float>(0) = 0;
		gyroFilter.statePost.at<float>(1) = 0;
		gyroFilter.statePost.at<float>(2) = 0;
		gyroFilter.statePost.at<float>(3) = 0;
		gyroFilter.statePost.at<float>(4) = 0;
		gyroFilter.statePost.at<float>(5) = 0;
		measurement = cv::Mat(3, 1, CV_32FC(1));

	}
	inline void updateSensorData(const pses_basis::SensorData& sensorData) {
		this->sensorData = sensorData;
		calcDt(sensorData.header.stamp, oldTimeStamp);
		oldTimeStamp = sensorData.header.stamp;
		calcSpeed();
		updateGyroMeasurements();
		updateGyroFilter();
		roll = estimatedGyro.at<float>(0);
		pitch = estimatedGyro.at<float>(1);
		yaw = estimatedGyro.at<float>(2);
		calcDeltaDistance();
	  	calcDrivenDistance();
		calcPosition();
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
	double speed;
	double drivenDistance;
	double deltaDistance;
	int drivingDirection; // -1 = backwards; 0 = stop; 1 = forwards
	ros::Time oldTimeStamp;
	geometry_msgs::Point position;
	ForwardKinematics odometric; // object needed for odometric calculations
	cv::KalmanFilter gyroFilter;
	cv::Mat measurement;
	cv::Mat estimatedGyro;
	pses_basis::SensorData sensorData;
	pses_basis::Command command;

	inline void calcDt(const ros::Time& currentTimeStamp, const ros::Time& oldTimeStamp) {
		dt = (currentTimeStamp - oldTimeStamp).toSec();
	}
	inline void calcSpeed() {
		speed =  isnan(sensorData.hall_sensor_dt)? 0.0 : drivingDirection * DRIVEN_DISTANCE_PER_TICK / sensorData.hall_sensor_dt;
	}
	inline void updateGyroMeasurements(){
		measurement.at<float>(0) = degToRad(sensorData.angular_velocity_x);
    	measurement.at<float>(1) = degToRad(sensorData.angular_velocity_y);
    	measurement.at<float>(2) = degToRad(sensorData.angular_velocity_z);
	}
	inline void updateGyroFilter(){
		gyroFilter.transitionMatrix.at<float>(0,3) = dt;
		gyroFilter.transitionMatrix.at<float>(1,4) = dt;
		gyroFilter.transitionMatrix.at<float>(2,5) = dt;
		gyroFilter.predict();
		estimatedGyro = gyroFilter.correct(measurement);
	}
	inline void calcDeltaDistance() {
		deltaDistance = isnan(sensorData.hall_sensor_dt)?0.0 : drivingDirection*DRIVEN_DISTANCE_PER_TICK;
	}
	inline void calcDrivenDistance() {
		drivenDistance += deltaDistance;
	}
	inline void calcPosition() {
		std::vector<double> pos = odometric.getUpdateWithGyro(yaw, deltaDistance);
		position.x = pos.at(1);
		position.y = -pos.at(0);
		position.z = 0.0;
	}
	inline float degToRad(const float value) {
		return value*CV_PI/180;
	}
};

#endif
