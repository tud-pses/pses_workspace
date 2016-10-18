#ifndef DASHBOARD_H
#define DASHBOARD_H

#include <QMainWindow>
#include <ros/ros.h>
#include <QTimer>
#include <QKeyEvent>
#include <pses_basis/Command.h>
#include <nav_msgs/Odometry.h>

typedef pses_basis::Command command_data;
typedef nav_msgs::Odometry odometry_data;

namespace Ui {
class Dashboard;
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom, Ui::Dashboard* ui);

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
        odometry_data odom;

        ros::Publisher robotCommand;
        ros::Subscriber robotOdometry;
        QTimer *timer;

private slots:
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
