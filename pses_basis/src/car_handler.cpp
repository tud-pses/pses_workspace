#include <ros/ros.h>
#include <car_handler/SerialCommunication.h>
#include <std_msgs/String.h>
#include <pses_basis/SensorData.h>
#include <pses_basis/Command.h>


void commandCallback(const pses_basis::Command::ConstPtr& cmd, SerialCommunication* sc) {
    bool sent = false;
    if(sc->isOpen()) sent = sc->send(*cmd);
    //if (!sent) ROS_INFO_STREAM("Message: '" << *cmd << "' couldn't be sent to the microcontroller!");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_handler");
    ros::NodeHandle nh;

    SerialCommunication sc(115200, "ttyS0");

    ros::Subscriber command_sub = nh.subscribe<pses_basis::SensorData>("command", 10, std::bind(commandCallback, std::placeholders::_1, &sc));

    bool isOpen = sc.openConnection();
    std::string config = "!DAQ GRP 1 ~ALL=20 USF USL USR";
    if(sc.isOpen()) sc.send(config);
    ros::Duration(0.1).sleep();
    config = "!DAQ START";
    if(sc.isOpen()) sc.send(config);
    ros::Duration(0.1).sleep();


    pses_basis::SensorData out;

    ros::Rate loop_rate(100);
    while(ros::ok()) {

    bool received = sc.receive(out);
    if(received) ROS_INFO_STREAM("Received: " << out);
    
    ros::spinOnce();
    loop_rate.sleep();

    }

ros::spin();

}
