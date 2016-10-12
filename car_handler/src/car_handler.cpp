#include <ros/ros.h>
#include <car_handler/car_handler.h>
#include <car_handler/SerialCommunication.h>
#include <std_msgs/String.h>


void commandCallback(const std_msgs::String::ConstPtr& cmd, SerialCommunication* sc) {
    bool sent = false;
    if(sc->isOpen()) sent = sc->send(cmd->data);
    if (!sent) ROS_INFO_STREAM("Message: '" << cmd->data << "' couldn't be sent to the microcontroller!");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_handler");

    ros::NodeHandle nh;

    SerialCommunication sc(115200, "ttyS0");

    ros::Subscriber command_sub = nh.subscribe<std_msgs::String>("command", 10, std::bind(commandCallback, std::placeholders::_1, &sc));

    bool isOpen = sc.openConnection();
    if(sc.isOpen()) sc.setSensorGroup({SC::rangeSensorLeft, SC::rangeSensorRight, SC::rangeSensorFront});
    ros::Duration(0.01).sleep();
    if(sc.isOpen()) sc.startSensors();
    ros::Duration(0.01).sleep();

    if(sc.isOpen()) sc.setSteeringLevel(25);
    ros::Duration(1.00).sleep();
    if(sc.isOpen()) sc.setSteeringLevel(-25);
    ros::Duration(1.00).sleep();
    if(sc.isOpen()) sc.setSteeringLevel(0);
    ros::Duration(1.00).sleep();

    if(sc.isOpen()) sc.setMotorLevel(20);
    ros::Duration(1.00).sleep();
    if(sc.isOpen()) sc.setMotorLevel(0);
    ros::Duration(1.00).sleep();
    if(sc.isOpen()) sc.setMotorLevel(-20);
    ros::Duration(1.00).sleep();
    if(sc.isOpen()) sc.setMotorLevel(0);
    ros::Duration(1.00).sleep();


    std::string out;

    ros::Rate loop_rate(400);
    while(ros::ok()) {

    bool received = sc.receive(out);
    if(received) ROS_INFO_STREAM("Received: "<< out);
    
    ros::spinOnce();
    loop_rate.sleep();

    }

ros::spin();

if(sc.isOpen()) sc.stopSensors();
ros::Duration(0.01).sleep();

}
