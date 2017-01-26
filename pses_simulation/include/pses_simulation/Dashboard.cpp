#include "Dashboard.h"
#include "ui_Dashboard.h"

Dashboard::Dashboard(ros::NodeHandle* nh, QWidget *parent) :
        QMainWindow(parent), ui(new Ui::Dashboard), nh(nh){
          
        ui->setupUi(this);
        modeControl =  nh->advertise<string_msg>("pses_basis/mode_control", 10);
        robotCommand = nh->advertise<command_data>("pses_basis/command", 10);
        robotOdometry = nh->subscribe<odometry_data>("odom", 10, boost::bind(odometryCallback, _1, ui));
        robotSensors = nh->subscribe<sensor_data>("pses_basis/sensor_data", 10, boost::bind(sensorCallback, _1, ui));
        carInfo = nh->subscribe<info_data>("pses_basis/car_info", 10, boost::bind(infoCallback, _1, ui));

        connect(ui->speedSlider, SIGNAL(valueChanged(int)), this, SLOT(valueChangedSpeed(int)));
        connect(ui->steeringSlider, SIGNAL(valueChanged(int)), this, SLOT(valueChangedSteering(int)));
        connect(ui->maxSpeed, SIGNAL(clicked()), this, SLOT(maxSpeedClicked()));
        connect(ui->minSpeed, SIGNAL(clicked()), this, SLOT(minSpeedClicked()));
        connect(ui->zeroSpeed, SIGNAL(clicked()), this, SLOT(zeroSpeedClicked()));
        connect(ui->maxSteering, SIGNAL(clicked()), this, SLOT(maxSteeringClicked()));
        connect(ui->minSteering, SIGNAL(clicked()), this, SLOT(minSteeringClicked()));
        connect(ui->centerSteering, SIGNAL(clicked()), this, SLOT(centerSteeringClicked()));

        ui->modeSelection->addItem(QString("Remote Control"), QVariant());
        ui->modeSelection->addItem(QString("Follow Wall"), QVariant());
        ui->modeSelection->addItem(QString("Roundtrip w. Obstacles"), QVariant());
        ui->modeSelection->addItem(QString("Park Car"), QVariant());
        ui->modeSelection->addItem(QString("Lane Detection"), QVariant());
        ui->modeSelection->addItem(QString("Exploration"), QVariant());

        connect(ui->modeSelection, SIGNAL(currentIndexChanged(int)), this, SLOT(modeSelect(int)));

        timer = new QTimer(this);
        connect(timer, SIGNAL(timeout()), this, SLOT(pollNodeHandle()));
        timer->start(50);

}

Dashboard::~Dashboard(){
        delete ui;
}

void odometryCallback(const odometry_data::ConstPtr& odom, Ui::Dashboard* ui){
        // absolute velocity components of robot
        ui->odom_v_x->display(odom->twist.twist.linear.x);
        ui->odom_v_y->display(odom->twist.twist.linear.y);
        // absolute position components of robot
        ui->odom_p_x->display(odom->pose.pose.position.x);
        ui->odom_p_y->display(odom->pose.pose.position.y);
        ros::spinOnce();
}

void sensorCallback(const sensor_data::ConstPtr& sensor, Ui::Dashboard* ui){
        // ultra sound range sensor data
        ui->sensor_usl->display(sensor->range_sensor_left);
        ui->sensor_usf->display(sensor->range_sensor_front);
        ui->sensor_usr->display(sensor->range_sensor_right);
        // hall sensor data
        ui->sensor_hall_dt->display(sensor->hall_sensor_dt);
        ui->sensor_hall_dtf->display(sensor->hall_sensor_dt_full);
        ui->sensor_hall_count->display((int)sensor->hall_sensor_count); // <- this is an unsinged integer
        // battery sensor data
        ui->sensor_batt_sys->display(sensor->system_battery_voltage);
        ui->sensor_batt_motor->display(sensor->motor_battery_voltage);
        // acceleration sensor
        ui->sensor_ax->display(sensor->accelerometer_x);
        ui->sensor_ay->display(sensor->accelerometer_y);
        ui->sensor_az->display(sensor->accelerometer_z);
        // gyroscope sensor (angular velocity arround x-/y-/z-axis)
        ui->sensor_wx->display(sensor->angular_velocity_x/M_PI*180.0);
        ui->sensor_wy->display(sensor->angular_velocity_y/M_PI*180.0);
        ui->sensor_wz->display(sensor->angular_velocity_z/M_PI*180.0);
        ros::spinOnce();
}

void infoCallback(const info_data::ConstPtr& info, Ui::Dashboard* ui){
        // relative velocity of robot (velocity on current trajectory)
        ui->info_v_all->display(info->speed);
        // over all driven distance
        ui->info_d_all->display(info->driven_distance);
        // rpy-Info
        ui->info_roll->display(info->roll/M_PI*180.0);
        ui->info_pitch->display(info->pitch/M_PI*180.0);
        ui->info_yaw->display(info->yaw/M_PI*180.0);
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
                if(steering < 50) {
                        cmd.steering_level = steering + 5;
                }
                break;
        }


        case Qt::Key_D: {
                if(steering > -50) {
                        cmd.steering_level  = steering - 5;
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
        timer->start(50);
}

void Dashboard::modeSelect(int index){
        mode.data = ui->modeSelection->itemText(index).toStdString();
        modeControl.publish(mode);
        ros::spinOnce();
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
