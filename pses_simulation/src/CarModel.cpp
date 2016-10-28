/*
 * CarModel.cpp
 *
 *  Created on: 11.05.2016
 *      Author: Basti
 */

#include <pses_simulation/CarModel.h>

CarModel::CarModel(const double dAxis, const ros::Time& time, const double vMax, const double angleMax,
                   const double speedMax, const double steeringMax) : lastUpdate(time), vMax(vMax),
                    angleMax(angleMax), speedMax(speedMax), steeringMax(steeringMax) {
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
                                timeStep = (time-lastUpdate).toSec();
								lastUpdate = time;
                                double ds = velocity * timeStep;
								double oldYaw = pose[2];
								pose = fwdKin.getUpdate(fwdKin.degToRad(steeringAngle), ds);
								distance += std::fabs(ds);
                                double prevV = velocity;
								speedToVelocity(newSpeed);
                                setAngularVelocity(oldYaw);
                                setSteering(newSteering);
                                setVelocityComponents();
                                setAccelerationComponents(prevV);
								return std::make_shared<std::vector<double> >(pose);
}

const pose_ptr CarModel::getUpdateTwist(const twist_msg cmdVel, const ros::Time& time){
                                timeStep = (time-lastUpdate).toSec();
								lastUpdate =time;
								double ds = velocity * timeStep;
								double oldYaw = pose[2];
								pose = fwdKin.getUpdate(fwdKin.degToRad(steeringAngle), ds);
								distance += std::fabs(ds);
                                double prevV = velocity;
                                setVelocity(cmdVel.linear.x);
                                setAngularVelocity(oldYaw);
                                angleToSteering(fwdKin.radToDeg(cmdVel.angular.z));
                                setVelocityComponents();
                                setAccelerationComponents(prevV);
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
                                if(alpha>angleMax){
                                    setSteering(steeringMax);
                                }else if(alpha<-angleMax){
                                    setSteering(-steeringMax);
                                }else{
                                    setSteering((alpha/angleMax)*steeringMax);
                                }
}
void CarModel::setVelocity(const double newVel){
                                if(newVel>vMax){
                                    speedToVelocity(speedMax);
                                }else if(newVel<-vMax){
                                    speedToVelocity(-speedMax);
                                }else{
                                    speedToVelocity((newVel/vMax)*speedMax);
                                }
}

void CarModel::speedToVelocity(const int speed) {
                                velocity = (speed/speedMax)*vMax;
}
void CarModel::setAngularVelocity(const double oldYaw){
                                double dTh = pose[2]-oldYaw;
								if(dTh<-fwdKin.PI) {
																dTh += 2*fwdKin.PI;
								}else if(dTh>fwdKin.PI) {
																dTh -= 2*fwdKin.PI;
								}
								angularVelocity = dTh/timeStep;
}
void CarModel::setSteering(const int steering){
                                if(steering>steeringMax) {
                                                                this->steering = steeringMax;
																steeringToAngle();
                                }else if(steering < -steeringMax) {
                                                                this->steering = -steeringMax;
																steeringToAngle();
								}else{
																this->steering = steering;
																steeringToAngle();
								}
}
void CarModel::steeringToAngle(){
                                steeringAngle = (steering/steeringMax)*angleMax;
}
void CarModel::setVelocityComponents(){
                                v_x = velocity*std::cos(pose[2]);
                                v_y = velocity*std::sin(pose[2]);
}
void CarModel::setAccelerationComponents(const double prevV){
                                a_x = (velocity-prevV)/timeStep;
                                if(steeringAngle==0){
                                    a_y=0;
                                }else{
                                    double sign = (steeringAngle>0)?1:-1;
                                    a_y = sign*velocity*velocity/fwdKin.getRadius();
                                }
}
