#include <ros/ros.h>
#include <iostream>
#include <pses_simulation/CarModel.h>
#include <vector>
#include <string>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include <tf/transform_broadcaster.h>
#include <pses_basis/Command.h>
#include <pses_basis/CarInfo.h>
#include <pses_basis/SensorData.h>

void motionCommands(const geometry_msgs::Twist::ConstPtr& motionIn, geometry_msgs::Twist* motionOut, bool* flag){
        *motionOut = *motionIn;
        *flag = true;
}

void controlCommands(const pses_basis::Command::ConstPtr& cmdIn, pses_basis::Command* cmdOut, bool* flag){
        *cmdOut = *cmdIn;
        *flag = false;
}

void ultraSoundSensor(const sensor_msgs::Range::ConstPtr& usSensor, pses_basis::SensorData* sensors){
        std::string sensor_id = usSensor->header.frame_id;
        if(sensor_id.compare("front_sensor")==0){
          sensors->range_sensor_front=usSensor->range;
        }else if(sensor_id.compare("left_sensor")==0){
          sensors->range_sensor_left=usSensor->range;
        }else{
          sensors->range_sensor_right=usSensor->range;
        }
}

int main(int argc, char **argv){

        ros::init(argc, argv, "simulation_control");
        ros::NodeHandle nh;
        ros::Time currentTime = ros::Time::now();

        CarModel car(0.25, currentTime);
        std::vector<double>simPose;
        bool localPlannerFlag = true;

        pses_basis::Command control;
        geometry_msgs::Twist motion;
        pses_basis::CarInfo info;
        pses_basis::SensorData sensors;

        tf::TransformBroadcaster odom_broadcaster;
        geometry_msgs::TransformStamped odom_trans;
        nav_msgs::Odometry odom;

        ros::Subscriber motionSub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10, std::bind(motionCommands, std::placeholders::_1, &motion, &localPlannerFlag));
        ros::Subscriber commandSub = nh.subscribe<pses_basis::Command>("pses_basis/command", 10, std::bind(controlCommands, std::placeholders::_1, &control, &localPlannerFlag));
        ros::Subscriber uslSub = nh.subscribe<sensor_msgs::Range>("left_us_range", 10, std::bind(ultraSoundSensor, std::placeholders::_1, &sensors));
        ros::Subscriber usrSub = nh.subscribe<sensor_msgs::Range>("right_us_range", 10, std::bind(ultraSoundSensor, std::placeholders::_1, &sensors));
        ros::Subscriber usfSub = nh.subscribe<sensor_msgs::Range>("front_us_range", 10, std::bind(ultraSoundSensor, std::placeholders::_1, &sensors));
        ros::Publisher odomPub = nh.advertise<nav_msgs::Odometry>("odom", 10);
        ros::Publisher carInfoPub = nh.advertise<pses_basis::CarInfo>("pses_basis/car_info", 10);
        ros::Publisher sensorPub = nh.advertise<pses_basis::SensorData>("pses_basis/sensor_data", 10);

        // Loop starts here:
        ros::Rate loop_rate(100);
        while(ros::ok()) {
                currentTime = ros::Time::now();
                if(!localPlannerFlag) {
                        simPose = *car.getUpdate(control.steering_level, control.motor_level, currentTime);
                }else{
                        simPose = *car.getUpdateTwist(motion, currentTime);
                }

                geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(simPose[2]);

                // publish the transform over tf
                odom_trans.header.stamp = currentTime;
                odom_trans.header.frame_id = "odom";
                odom_trans.child_frame_id = "base_footprint";
                odom_trans.header.seq++;

                odom_trans.transform.translation.x = simPose[1];
                odom_trans.transform.translation.y = -simPose[0];
                odom_trans.transform.translation.z = 0;  // misuse translation along z-axis for displaying total driven distance
                odom_trans.transform.rotation = odom_quat;
                // send the transform
                odom_broadcaster.sendTransform(odom_trans);

                // set the odometry message
                odom.header.stamp = currentTime;
                odom.header.frame_id = "odom";
                odom.header.seq++;
                // set the position
                odom.pose.pose.position.x = simPose[1];
                odom.pose.pose.position.y = -simPose[0];
                odom.pose.pose.position.z = 0;
                odom.pose.pose.orientation = odom_quat;
                // set the velocity
                odom.child_frame_id = "base_footprint";
                odom.twist.twist.linear.x = car.getVx();
                odom.twist.twist.linear.y = car.getVy();
                odom.twist.twist.linear.z = 0;
                odom.twist.twist.angular.z = car.getAngularVelocity();

                // set car_info message
                info.header.stamp = currentTime;
                info.header.frame_id = "odom";
                info.header.seq++;
                info.header.car_id = 0;
                // set rpy-Angles
                info.roll=0;
                info.pitch=0;
                info.yaw=simPose[2];
                // set driven distance
                info.driven_distance = car.getDistance();
                // set speed
                info.speed = car.getVelocity();

                // set sensor data message
                sensors.header.stamp = currentTime;
                sensors.header.frame_id = "odom";
                sensors.header.seq++;
                sensors.header.car_id = 0;
                // set gyro data (deg/s)
                sensors.angular_velocity_z = car.getAngularVelocity();
                // set accelerometer data
                sensors.accelerometer_x = car.getAx();
                sensors.accelerometer_y = car.getAy();

                // publish the messages over ROS
                odomPub.publish(odom);
                carInfoPub.publish(info);
                sensorPub.publish(sensors);

                ros::spinOnce();
                loop_rate.sleep();
        }

        ros::spin();
}
