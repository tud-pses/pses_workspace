#include <ros/ros.h>
#include <pses_basis/SerialCommunication.h>
#include <pses_basis/SensorData.h>
#include <pses_basis/Command.h>

void commandCallback(const pses_basis::Command::ConstPtr& cmd, SerialCommunication* sc) {
    sc->sendCommand(*cmd);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_handler");
    ros::NodeHandle nh;

    SerialCommunication sc(921600, "ttyUSB0");

    ros::Subscriber command_sub = nh.subscribe<pses_basis::Command>("pses_basis/command", 10, std::bind(commandCallback, std::placeholders::_1, &sc));
    ros::Publisher sensor_pub = nh.advertise<pses_basis::SensorData>("pses_basis/sensor_data", 10);

    sc.initUCBoard();

    pses_basis::SensorData sensorValues;

    ros::Rate loop_rate(400);
    while(ros::ok()) {

    if(sc.getSensorData(sensorValues)) sensor_pub.publish(sensorValues);

    ros::spinOnce();
    loop_rate.sleep();

    }

    ros::spin();

    sc.deactivateUCBoard();

}
