/*
 * CarModel.h
 *
 *  Created on: 11.05.2016
 *      Author: Basti
 */

#ifndef CARMODEL_H_
#define CARMODEL_H_

#include "ForwardKinematics.h"
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <memory.h>
#include <ros/ros.h>

typedef geometry_msgs::Twist twist_msg;
typedef std::shared_ptr<std::vector<double> > pose_ptr;

class CarModel {
public:
                                CarModel(const double dAxis, const ros::Time& time,
                                         const double vMax=2.0, const double angleMax=22.5,
                                         const double speedMax = 20, const double steeringMax = 50);

								const pose_ptr getUpdate(const int newSteering, const int newSpeed, const ros::Time& time);
								const pose_ptr getUpdateTwist(const twist_msg cmd_vel, const ros::Time& time);

								const int getSteering() const;
								const double getSteeringAngle() const;
								const double getTimeStep() const;
								const double getVelocity() const;
								const double getAngularVelocity() const;
								const double getDistance() const;
								const double getVx() const;
								const double getVy() const;
								const double getAx() const;
								const double getAy() const;
private:
								ForwardKinematics fwdKin;
								ros::Time lastUpdate;
                                double vMax;
                                double angleMax;
                                double speedMax;
                                double steeringMax;
								int steering;
								double steeringAngle;
								double timeStep;
								double velocity;
								double distance;
								std::vector<double> pose;
								double angularVelocity;
								double v_x;
								double v_y;
								double a_x;
								double a_y;

								void steeringToAngle();
								void angleToSteering(const double alpha);
								void setSteering(const int steering);
                                void setVelocity(const double newVel);
								void speedToVelocity(const int speed);
                                void setAngularVelocity(const double oldYaw);
                                void setVelocityComponents();
                                void setAccelerationComponents(const double prevV);
};

#endif /* CARMODEL_H_ */
