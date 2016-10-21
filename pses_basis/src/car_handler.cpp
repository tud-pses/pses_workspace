#include <ros/ros.h>
#include <pses_basis/SerialCommunication.h>
#include <pses_basis/SensorData.h>
#include <pses_basis/Command.h>
#include <pses_basis/PsesUcBoard.h>

void commandCallback(const pses_basis::Command::ConstPtr& cmd, SerialCommunication* sc) {
    sc->sendCommand(*cmd);
}

/*
void commandCallback(const pses_basis::Command::ConstPtr& cmd, PsesUcBoard* board) {
    //sc->sendCommand(*cmd);
    try{
        board->setSteering(cmd->steering_level);
    }catch(std::exception& e){
        ROS_ERROR("%s",e.what());
    }
    try{
        board->setMotor(cmd->motor_level);
    }catch(std::exception& e){
        ROS_ERROR("%s",e.what());
    }
}
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_handler");
    ros::NodeHandle nh;

    SerialCommunication sc(921600, "ttyUSB0");
    //PsesUcBoard board(921600, "ttyUSB0");

    ros::Subscriber command_sub = nh.subscribe<pses_basis::Command>("pses_basis/command", 10, std::bind(commandCallback, std::placeholders::_1, &sc));
    //ros::Subscriber command_sub = nh.subscribe<pses_basis::Command>("pses_basis/command", 10, std::bind(commandCallback, std::placeholders::_1, &board));
    ros::Publisher sensor_pub = nh.advertise<pses_basis::SensorData>("pses_basis/sensor_data", 10);

    sc.initUCBoard();
    /*
    try{
        board.initUcBoard();
    }catch(std::exception& e){
        ROS_ERROR("%s",e.what());
    }
    */
    pses_basis::SensorData sensorValues;

    ros::Rate loop_rate(400);
    while(ros::ok()) {


    if(sc.getSensorData(sensorValues)) sensor_pub.publish(sensorValues);
        /*
    try{
        board.emptyAllStacks();
    }catch(std::exception& e){
        ROS_ERROR("%s",e.what());
    }
    */

    ros::spinOnce();
    loop_rate.sleep();

    }

    ros::spin();
    //board.deactivateUCBoard();
    sc.deactivateUCBoard();

}
