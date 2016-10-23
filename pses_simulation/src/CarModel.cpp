/*
 * CarModel.cpp
 *
 *  Created on: 11.05.2016
 *      Author: Basti
 */

#include <pses_simulation/CarModel.h>

CarModel::CarModel(const double dAxis, const ros::Time& time) : lastUpdate(time) {
								// init kinematic model
								fwdKin = ForwardKinematics(dAxis);
								// set steering to quasi neutral
								setSteering(8);
								// start with stationary state
								velocity = 0;
								distance = 0;
								timeStep = 0;
								pose=std::vector<double>(3,0);
								angularVelocity = 0;

}

const pose_ptr CarModel::getUpdate(const int newSteering, const int newSpeed, const ros::Time& time){
								timeStep = (time-lastUpdate).toNSec()/NSEC_PER_SEC;
								//ROS_INFO("timestep: %lf", timeStep);
								lastUpdate = time;
								double ds = velocity * timeStep;
								//double ds = speedToVelocity(newSpeed) * timeStep;
								double oldYaw = pose[2];
								pose = fwdKin.getUpdate(fwdKin.degToRad(steeringAngle), ds);
								distance += std::fabs(ds);
								speedToVelocity(newSpeed);
								setAngularVelocity(oldYaw, pose[2]);
								setSteering(newSteering);
								double prevVx = v_x;
								double prevVy = v_y;
								setVelocityComponents(velocity, pose[2]);
								setAccelerationComponents(prevVx, prevVy, pose[2], timeStep);
								return std::make_shared<std::vector<double> >(pose);
}

const pose_ptr CarModel::getUpdateTwist(const twist_msg cmdVel, const ros::Time& time){
								timeStep = (time-lastUpdate).toNSec()/NSEC_PER_SEC;
								lastUpdate =time;
								double ds = velocity * timeStep;
								double oldYaw = pose[2];
								pose = fwdKin.getUpdate(fwdKin.degToRad(steeringAngle), ds);
								distance += std::fabs(ds);
								velocity = cmdVel.linear.x;
								setAngularVelocity(oldYaw, pose[2]);
								angleToSteering(fwdKin.radToDeg(cmdVel.angular.z));
								double prevVx = v_x;
								double prevVy = v_y;
								setVelocityComponents(velocity, pose[2]);
								setAccelerationComponents(prevVx, prevVy, pose[2], timeStep);
								return std::make_shared<std::vector<double> >(pose);
}

const double CarModel::getTimeStep() const {
								return timeStep;
}

const double CarModel::getDistance() const {
								return distance;
}

const double CarModel::getVelocity() const {
								return velocity;
}

const int CarModel::getSteering() const {
								return steering;
}
const double CarModel::getSteeringAngle() const {
								return steeringAngle;
}

const double CarModel::getAngularVelocity() const {
								return angularVelocity;
}

const double CarModel::getVx() const{
								return v_x;
}
const double CarModel::getVy() const{
								return v_y;
}
const double CarModel::getAx() const{
								return a_x;
}
const double CarModel::getAy() const{
								return a_y;
}

void CarModel::angleToSteering(const double alpha){
								if(alpha>=angleArray[0]) {
																setSteering(-50);
																return;
								}else if(alpha<=angleArray[100]) {
																setSteering(50);
																return;
								}else{
																for(int i = 1; i <= 100; i++) {
																								if(alpha < angleArray[i-1] && alpha >= angleArray[i]) {
																																setSteering(i-50);
																																return;
																								}
																}
								}
}

void CarModel::speedToVelocity(const int speed) {
								if(speed<=-10) {
																velocity = velocityArray[0];
								}else if(speed>=10) {
																velocity = velocityArray[20];
								}else{
																velocity = velocityArray[10+speed];
								}
}



void CarModel::setAngularVelocity(const double yaw0, const double yaw1){
								double dTh = yaw1-yaw0;
								if(dTh<-fwdKin.PI) {
																dTh += 2*fwdKin.PI;
								}else if(dTh>fwdKin.PI) {
																dTh -= 2*fwdKin.PI;
								}
								angularVelocity = dTh/timeStep;
}

void CarModel::setSteering(const int steering){
								if(steering>50) {
																this->steering = 50;
																steeringToAngle();
								}else if(steering < -50) {
																this->steering = -50;
																steeringToAngle();
								}else{
																this->steering = steering;
																steeringToAngle();
								}
}


void CarModel::steeringToAngle(){
								//steeringAngle = angleArray[steering+50];
								steeringAngle = (steering/50.0)*22.5;
}

void CarModel::setVelocityComponents(const double velocity, const double yaw){
							  	v_x = velocity*std::cos(yaw);
								v_y = velocity*std::sin(yaw);
}

void CarModel::setAccelerationComponents(const double prevVx, const double prevVy, const double yaw, const double dT){
							  	a_x = (prevVx-v_x)/dT;
								a_y = (prevVy-v_y)/dT;
}
