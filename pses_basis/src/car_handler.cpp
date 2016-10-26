#include <ros/ros.h>
#include <pses_basis/SerialCommunication.h>
#include <pses_basis/SensorData.h>
#include <pses_basis/Command.h>
#include <pses_basis/PsesUcBoard.h>

void getSensorData(PsesUcBoard& board, pses_basis::SensorData& sensorValues){
    try{
        board.getSensorData(sensorValues);
    }catch(std::exception& e){
    }
}
void checkBoardErrors(PsesUcBoard& board){
    try{
        if(board.boardErrors()){
            std::string msg;
            board.getBoardError(msg);
            ROS_ERROR("%s",msg.c_str());
        }
    }catch(std::exception& e){
        ROS_ERROR("%s",e.what());
    }
}

void checkBoardMessages(PsesUcBoard& board){
    try{
        if(board.boardMessages()){
            std::string msg;
            board.getBoardMessage(msg);
            ROS_INFO("%s",msg.c_str());
        }
    }catch(std::exception& e){
        ROS_ERROR("%s",e.what());
    }
}

void commandCallback(const pses_basis::Command::ConstPtr& cmd, PsesUcBoard* board) {
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
    try{
        if(cmd->enable_kinect){
            board->activateKinect();
        }else{
            board->deactivateKinect();
        }
    }catch(std::exception& e){
        ROS_ERROR("%s",e.what());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_handler");
    ros::NodeHandle nh;

    PsesUcBoard board;

    ros::Subscriber command_sub = nh.subscribe<pses_basis::Command>("pses_basis/command", 10, std::bind(commandCallback, std::placeholders::_1, &board));
    ros::Publisher sensor_pub = nh.advertise<pses_basis::SensorData>("pses_basis/sensor_data", 10);

    try{
        board.initUcBoard();
    }catch(std::exception& e){
        ROS_ERROR("%s",e.what());
    }

    checkBoardMessages(board);
    checkBoardErrors(board);

    pses_basis::SensorData sensorValues;

    ros::Rate loop_rate(200);
    while(ros::ok()) {

    checkBoardErrors(board);
    checkBoardMessages(board);
    getSensorData(board, sensorValues);
    sensor_pub.publish(sensorValues);

    ros::spinOnce();
    loop_rate.sleep();

    }

    ros::spin();
    board.deactivateUCBoard();

}
