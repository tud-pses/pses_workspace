#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <pses_basis/SensorData.h>
#include <pses_basis/Command.h>
#include <pses_basis/OdometryHelper.h>
#include <pses_basis/CarInfo.h>

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
	odom.twist.twist.linear.x = odomHelper.getSpeed()*std::cos(sensorData.angular_velocity_z);
	odom.twist.twist.linear.y = odomHelper.getSpeed()*std::sin(sensorData.angular_velocity_z);
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

void dataCallback(const pses_basis::SensorData::ConstPtr& sensorData, tf::TransformBroadcaster* odomBroadcaster,
								ros::Publisher* odom_pub, OdometryHelper* odomHelper, ros::Publisher* carInfo_pub) {
	
	pses_basis::SensorData sensorDataCopy = *sensorData;
	// update sensor data for later odometric calculations
	odomHelper->updateSensorData(sensorDataCopy);
	// objects to store the odometry and its transform
	geometry_msgs::TransformStamped odomTransform;
	nav_msgs::Odometry odom;
	pses_basis::CarInfo info;
	// since all odometry is 6DOF we'll need a quaternion created from yaw
	geometry_msgs::Quaternion odomQuaternion;
	odomHelper->getQuaternion(odomQuaternion);
	//build odom transform for tf and the odometry message
	buildOdometryTransformation(odomTransform, sensorDataCopy, *odomHelper, odomQuaternion);
	buildOdometryMessage(odom, sensorDataCopy, *odomHelper, odomQuaternion);
	buildInfoMessage(info, sensorDataCopy, *odomHelper);
	// send the transform
	odomBroadcaster->sendTransform(odomTransform);
	// publish the odometry message
	odom_pub->publish(odom);
	carInfo_pub->publish(info);
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
	ros::Publisher carInfo_pub = nh.advertise<pses_basis::CarInfo>("pses_basis/car_info", 50);
	// Here we subscribe to the sensor_data and command topics
	ros::Subscriber sensor_data_sub = nh.subscribe<pses_basis::SensorData>("pses_basis/sensor_data", 1,
							std::bind(dataCallback, std::placeholders::_1, &odomBroadcaster, &odom_pub, &odomHelper, &carInfo_pub));
	ros::Subscriber command_sub = nh.subscribe<pses_basis::Command>("pses_basis/command", 1, std::bind(commandCallback, std::placeholders::_1, &odomHelper));

	ros::spin();

	return 0;
}
