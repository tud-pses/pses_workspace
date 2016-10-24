#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <pses_basis/SensorData.h>
#include <pses_basis/Command.h>
#include <pses_basis/OdometryHelper.h>
#include <sensor_msgs/Imu.h>
#include <pses_basis/CarInfo.h>

void buildImuMessage(sensor_msgs::Imu& imu, const pses_basis::SensorData& sensorData) {

	imu.header.stamp = sensorData.header.stamp;
	imu.header.frame_id = "odom";
	imu.angular_velocity.x = sensorData.angular_velocity_x;
	imu.angular_velocity.y = sensorData.angular_velocity_y;
	imu.angular_velocity.z = sensorData.angular_velocity_z;
	imu.linear_acceleration.x = sensorData.accelerometer_x;
	imu.linear_acceleration.y = sensorData.accelerometer_y;
	imu.linear_acceleration.z = sensorData.accelerometer_z;
}

void buildOdometryTransformation(geometry_msgs::TransformStamped& odomTransform, const pses_basis::SensorData& sensorData,
								OdometryHelper& odomHelper, geometry_msgs::Quaternion& odomQuaternion) {

	odomTransform.header.stamp = sensorData.header.stamp;
	odomTransform.header.frame_id = "odom";
	odomTransform.child_frame_id = "base_footprint";
	geometry_msgs::Point position = odomHelper.getPosition();
	odomTransform.transform.translation.x = position.x;
	odomTransform.transform.translation.y = position.y;
	odomTransform.transform.translation.z = 0.0;
	odomTransform.transform.rotation = odomQuaternion;
}

void buildOdometryMessage(nav_msgs::Odometry& odom, const pses_basis::SensorData& sensorData,
								OdometryHelper& odomHelper, geometry_msgs::Quaternion& odomQuaternion) {

	odom.header.stamp = sensorData.header.stamp;
	odom.header.frame_id = "odom";

	// set the position
	odom.pose.pose.position = odomHelper.getPosition();
	odom.pose.pose.orientation = odomQuaternion;

	// set the velocity
	odom.child_frame_id = "base_footprint";
	odom.twist.twist.linear.x = odomHelper.getSpeed()*std::cos(odomHelper.getYaw());
	odom.twist.twist.linear.y = odomHelper.getSpeed()*std::sin(odomHelper.getYaw());
	odom.twist.twist.angular.z = sensorData.angular_velocity_z;
}

void buildInfoMessage(pses_basis::CarInfo& info, const pses_basis::SensorData& sensorData, OdometryHelper& odomHelper) {
	// configure header
	info.header.stamp = sensorData.header.stamp;
	info.header.frame_id = sensorData.header.frame_id;
	info.header.seq++;
	info.header.car_id = sensorData.header.car_id;
	// set rpy-Angles
    info.roll = odomHelper.getRoll();
    info.pitch = odomHelper.getPitch();
    info.yaw = odomHelper.getYaw();
	// set driven distance
	info.driven_distance = odomHelper.getDrivenDistance();
	// set speed
	info.speed = odomHelper.getSpeed();
}

void commandCallback(const pses_basis::Command::ConstPtr& cmd, OdometryHelper* odomHelper) {
	// update the command information for later odometric calculations
	odomHelper->updateCommand(*cmd);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu, OdometryHelper* odomHelper){
	sensor_msgs::Imu imu2 = *imu;
	odomHelper->updateIMUData(imu2);
}

void dataCallback(const pses_basis::SensorData::ConstPtr& sensorData, tf::TransformBroadcaster* odomBroadcaster, ros::Publisher* imu_pub,
								ros::Publisher* odom_pub, OdometryHelper* odomHelper, ros::Publisher* carInfo_pub) {
	// update sensor data for later odometric calculations
	odomHelper->updateSensorData(*sensorData);
	// objects to store the odometry and its transform
	geometry_msgs::TransformStamped odomTransform;
	nav_msgs::Odometry odom;
	pses_basis::CarInfo info;
	sensor_msgs::Imu imu;
	// since all odometry is 6DOF we'll need a quaternion created from yaw
	geometry_msgs::Quaternion odomQuaternion;
	odomHelper->getQuaternion(odomQuaternion);
	//build odom transform for tf and the odometry message
	buildOdometryTransformation(odomTransform, *sensorData, *odomHelper, odomQuaternion);
	buildOdometryMessage(odom, *sensorData, *odomHelper, odomQuaternion);
	buildInfoMessage(info, *sensorData, *odomHelper);
	buildImuMessage(imu, *sensorData);
	// send the transform
	odomBroadcaster->sendTransform(odomTransform);
	// publish the odometry message
	odom_pub->publish(odom);
	carInfo_pub->publish(info);
	imu_pub->publish(imu);
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "car_odometry");

	ros::NodeHandle nh;
	// object needed to send odometric information to the navigational stack
	tf::TransformBroadcaster odomBroadcaster;
	// object neeeded to store the current commands
	pses_basis::Command cmd;
	// objects needed for odometric calculations
	OdometryHelper odomHelper;
	// Publishes the results of the odometry calculations to other ros nodes
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
	ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 50);
	ros::Publisher carInfo_pub = nh.advertise<pses_basis::CarInfo>("pses_basis/car_info", 50);
	// Here we subscribe to the sensor_data and command topics
	ros::Subscriber sensor_data_sub = nh.subscribe<pses_basis::SensorData>("pses_basis/sensor_data", 1,
							std::bind(dataCallback, std::placeholders::_1, &odomBroadcaster, &imu_pub, &odom_pub, &odomHelper, &carInfo_pub));
	ros::Subscriber command_sub = nh.subscribe<pses_basis::Command>("pses_basis/command", 1, std::bind(commandCallback, std::placeholders::_1, &odomHelper));
	ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("imu/data", 50, std::bind(imuCallback, std::placeholders::_1, &odomHelper));

	ros::spin();

	return 0;
}
