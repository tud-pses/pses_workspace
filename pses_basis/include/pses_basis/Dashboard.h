#ifndef DASHBOARD_H
#define DASHBOARD_H

#include <QMainWindow>
#include <ros/ros.h>
#include <QTimer>
#include <QKeyEvent>
#include <pses_basis/Command.h>
#include <pses_basis/SensorData.h>
#include <nav_msgs/Odometry.h>
#include <pses_basis/CarInfo.h>
#include <math.h>
#include <std_msgs/String.h>
#include <string>
#include <sstream>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

typedef pses_basis::Command command_data;
typedef pses_basis::SensorData sensor_data;
typedef pses_basis::CarInfo info_data;
typedef nav_msgs::Odometry odometry_data;
typedef std_msgs::String string_msg;
typedef sensor_msgs::Image image_msg;


namespace Ui {
        class Dashboard;
}

void odometryCallback(const odometry_data::ConstPtr& odom, Ui::Dashboard* ui);
void sensorCallback(const sensor_data::ConstPtr& sensor, Ui::Dashboard* ui);
void infoCallback(const info_data::ConstPtr& info, Ui::Dashboard* ui);
void cameraCallback(const image_msg::ConstPtr& img, Ui::Dashboard* ui);
void depthCallback(const image_msg::ConstPtr& img, Ui::Dashboard* ui);

class Dashboard : public QMainWindow
{
        Q_OBJECT

public:
        explicit Dashboard(ros::NodeHandle* nh, QWidget *parent = 0);
        virtual ~Dashboard();
        void keyPressEvent(QKeyEvent *event);

private:
        Ui::Dashboard *ui;
        ros::NodeHandle* nh;
        command_data cmd;
        sensor_data sensor;
        odometry_data odom;
        info_data info;
        string_msg mode;

        ros::Publisher robotCommand;
        ros::Publisher modeControl;
        ros::Subscriber robotOdometry;
        ros::Subscriber robotSensors;
        ros::Subscriber carInfo;
        ros::Subscriber cameraSub;
        ros::Subscriber depthSub;
        QTimer *timer;

private slots:
        void toggleKinect();
        void modeSelect(int index);
        void cameraSelect(int index);
        void valueChangedSpeed(int value);
        void valueChangedSteering(int value);
        void maxSpeedClicked();
        void minSpeedClicked();
        void zeroSpeedClicked();
        void maxSteeringClicked();
        void minSteeringClicked();
        void centerSteeringClicked();
        void pollNodeHandle();
};

#endif // DASHBOARD_H
