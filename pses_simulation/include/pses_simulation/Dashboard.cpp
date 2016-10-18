#include "Dashboard.h"
#include "../../../build/pses_simulation/ui_Dashboard.h"

Dashboard::Dashboard(ros::NodeHandle* nh, QWidget *parent) :
        QMainWindow(parent), ui(new Ui::Dashboard), nh(nh)
{
        ui->setupUi(this);
        robotCommand = nh->advertise<command_data>("pses_basis/command", 10);
        robotOdometry = nh->subscribe<odometry_data>("odom", 10, boost::bind(odometryCallback, _1, ui));

        connect(ui->speedSlider, SIGNAL(valueChanged(int)), this, SLOT(valueChangedSpeed(int)));
        connect(ui->steeringSlider, SIGNAL(valueChanged(int)), this, SLOT(valueChangedSteering(int)));
        connect(ui->maxSpeed, SIGNAL(clicked()), this, SLOT(maxSpeedClicked()));
        connect(ui->minSpeed, SIGNAL(clicked()), this, SLOT(minSpeedClicked()));
        connect(ui->zeroSpeed, SIGNAL(clicked()), this, SLOT(zeroSpeedClicked()));
        connect(ui->maxSteering, SIGNAL(clicked()), this, SLOT(maxSteeringClicked()));
        connect(ui->minSteering, SIGNAL(clicked()), this, SLOT(minSteeringClicked()));
        connect(ui->centerSteering, SIGNAL(clicked()), this, SLOT(centerSteeringClicked()));

        timer = new QTimer(this);
        connect(timer, SIGNAL(timeout()), this, SLOT(pollNodeHandle()));
        timer->start(5);

}

Dashboard::~Dashboard()
{
        delete ui;
}

void odometryCallback(const odometry_data::ConstPtr& odom, Ui::Dashboard* ui){
        // relative velocity of robot (velocity on current trajectory)
        ui->odom_v_all->display(odom->twist.twist.linear.z);
        // absolute velocity components of robot
        ui->odom_v_x->display(odom->twist.twist.linear.x);
        ui->odom_v_y->display(odom->twist.twist.linear.y);
        // angular velocity arround z-axis of robot ("turnrate")
        ui->odom_w_z->display(odom->twist.twist.angular.z);
        // absolute position components of robot
        ui->odom_p_x->display(odom->pose.pose.position.x);
        ui->odom_p_y->display(odom->pose.pose.position.y);
        // over all driven distance
        ui->odom_d_all->display(odom->pose.pose.position.z);
        ros::spinOnce();
}

void Dashboard::keyPressEvent(QKeyEvent *event){
        cmd.header.stamp = ros::Time::now();
        int speed = cmd.motor_level;
        int steering = cmd.steering_level;

        switch ( event->key()) {
        case Qt::Key_W: {
                if(speed<20) {
                        cmd.motor_level = speed + 1;
                }
                break;
        }


        case  Qt::Key_S: {
                if(speed>-20) {
                        cmd.motor_level = speed - 1;
                }
                break;
        }


        case  Qt::Key_A: {
                if(steering > -50) {
                        cmd.steering_level = steering - 5;
                }
                break;
        }


        case Qt::Key_D: {
                if(steering < 50) {
                        cmd.steering_level  = steering + 5;
                }
                break;
        }

        case Qt::Key_Space: {
                cmd.motor_level  = 0;
                break;
        }

        }

        ui->speedSlider->setValue(cmd.motor_level);
        ui->steeringSlider->setValue(cmd.steering_level);

        robotCommand.publish(cmd);
        ros::spinOnce();
}

void Dashboard::pollNodeHandle(){
        ros::spinOnce();
        timer->start(5);
}

void Dashboard::valueChangedSpeed(int value){
        cmd.header.stamp = ros::Time::now();
        cmd.motor_level=value;
        robotCommand.publish(cmd);
        ros::spinOnce();
}

void Dashboard::valueChangedSteering(int value){
        cmd.header.stamp = ros::Time::now();
        cmd.steering_level=value;
        robotCommand.publish(cmd);
        ros::spinOnce();
}

void Dashboard::maxSpeedClicked(){
        cmd.header.stamp = ros::Time::now();
        cmd.motor_level=20;
        ui->speedSlider->setValue(20);
        robotCommand.publish(cmd);
        ros::spinOnce();
}

void Dashboard::minSpeedClicked(){
        cmd.header.stamp = ros::Time::now();
        cmd.motor_level=-20;
        ui->speedSlider->setValue(-20);
        robotCommand.publish(cmd);
        ros::spinOnce();
}

void Dashboard::zeroSpeedClicked(){
        cmd.header.stamp = ros::Time::now();
        cmd.motor_level=0;
        ui->speedSlider->setValue(0);
        robotCommand.publish(cmd);
        ros::spinOnce();
}

void Dashboard::maxSteeringClicked(){
        cmd.header.stamp = ros::Time::now();
        cmd.steering_level=50;
        ui->steeringSlider->setValue(50);
        robotCommand.publish(cmd);
        ros::spinOnce();
}

void Dashboard::minSteeringClicked(){
        cmd.header.stamp = ros::Time::now();
        cmd.steering_level=-50;
        ui->steeringSlider->setValue(-50);
        robotCommand.publish(cmd);
        ros::spinOnce();
}

void Dashboard::centerSteeringClicked(){
        cmd.header.stamp = ros::Time::now();
        cmd.steering_level=0;
        ui->steeringSlider->setValue(0);
        robotCommand.publish(cmd);
        ros::spinOnce();
}
