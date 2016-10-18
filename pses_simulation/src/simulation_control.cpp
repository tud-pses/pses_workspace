#include <ros/ros.h>
#include <iostream>
#include <pses_simulation/CarModel.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <pses_basis/Command.h>


geometry_msgs::Twist motion;
pses_basis::Command control;
std::vector<double>simPose;
int localPlannerFlag;


void motionCommands(const geometry_msgs::Twist::ConstPtr& motionCMD)
{
        motion = *motionCMD;
        localPlannerFlag = 1;
}

void controlCommands(const pses_basis::Command::ConstPtr& ctrlCMD)
{
        control = *ctrlCMD;
        localPlannerFlag = 0;
}

int main(int argc, char **argv){

        ros::init(argc, argv, "simulation_control");
        ros::NodeHandle nh;
        ros::Time currentTime = ros::Time::now();

        CarModel car(0.25, currentTime);
        motion = geometry_msgs::Twist();
        control = pses_basis::Command();
        tf::TransformBroadcaster odom_broadcaster;
        localPlannerFlag = 1;

        ros::Subscriber motionControl = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10, motionCommands);
        ros::Subscriber steeringControl = nh.subscribe<pses_basis::Command>("pses_basis/command", 10, controlCommands);
        ros::Publisher odomPub = nh.advertise<nav_msgs::Odometry>("odom", 10);

        // Loop starts here:
        ros::Rate loop_rate(100);
        while(ros::ok()) {
                currentTime = ros::Time::now();
                if(localPlannerFlag==0) {
                        simPose = *car.getUpdate(control.steering_level, control.motor_level, currentTime);
                }else{
                        simPose = *car.getUpdateTwist(motion, currentTime);
                }

                geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(simPose[2]);

                // first, we'll publish the transform over tf
                geometry_msgs::TransformStamped odom_trans;
                odom_trans.header.stamp = currentTime;
                odom_trans.header.frame_id = "odom";
                odom_trans.child_frame_id = "base_footprint";

                odom_trans.transform.translation.x = simPose[1];
                odom_trans.transform.translation.y = -simPose[0];
                odom_trans.transform.translation.z = 0;  // misuse translation along z-axis for displaying total driven distance
                odom_trans.transform.rotation = odom_quat;
                // send the transform
                odom_broadcaster.sendTransform(odom_trans);

                // next, we'll publish the odometry message over ROS
                nav_msgs::Odometry odom;
                odom.header.stamp = currentTime;
                odom.header.frame_id = "odom";
                // set the position
                odom.pose.pose.position.x = simPose[1];
                odom.pose.pose.position.y = -simPose[0];
                odom.pose.pose.position.z = car.getDistance();
                odom.pose.pose.orientation = odom_quat;
                // set the velocity
                odom.child_frame_id = "base_footprint";
                odom.twist.twist.linear.x = car.getVelocity()*std::cos(simPose[2]);
                odom.twist.twist.linear.y = car.getVelocity()*std::sin(simPose[2]);
                odom.twist.twist.linear.z = car.getVelocity(); // misuse linear motion along z-axis for displaying relative motion
                odom.twist.twist.angular.z = car.getAngularVelocity();


                // publish the messages
                odomPub.publish(odom);

                ros::spinOnce();
                loop_rate.sleep();
        }

        ros::spin();
}
