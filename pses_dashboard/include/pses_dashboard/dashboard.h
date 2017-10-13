#ifndef DASHBOARD_H
#define DASHBOARD_H

#include <math.h>
#include <string>
#include <sstream>

#include <qt5/QtWidgets/QMainWindow>
#include <qt5/QtCore/qtimer.h>
#include <qt5/QtGui/QKeyEvent>
#include <ui_dashboard.h>

#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Range.h>

#include <pses_basis/GetFirmwareVersion.h>
#include <pses_basis/GetControllerID.h>
#include <pses_basis/ToggleKinect.h>

// typedef pses_basis::Command command_data;
// typedef pses_basis::SensorData sensor_data;
// typedef pses_basis::CarInfo info_data;
// typedef nav_msgs::Odometry odometry_data;
typedef std_msgs::String string_msg;
typedef sensor_msgs::Image image_msg;
typedef sensor_msgs::Imu imu_msg;
typedef sensor_msgs::BatteryState battery_msg;
typedef sensor_msgs::MagneticField magnetic_msg;
typedef sensor_msgs::Range range_msg;
typedef std_msgs::Int16 int16_msg;
typedef std_msgs::Float64 float64_msg;
typedef std_msgs::UInt8 uint8_msg;

namespace Ui
{
class Dashboard;
}

class Dashboard : public QMainWindow
{
  Q_OBJECT

public:
  explicit Dashboard(ros::NodeHandle* nh, QWidget* parent = 0);
  virtual ~Dashboard();
  void keyPressEvent(QKeyEvent* event);

private:
  void imuCallback(const imu_msg::ConstPtr& imu);
  void magneticCallback(const magnetic_msg::ConstPtr& magnetic);
  void usrCallback(const range_msg::ConstPtr& usr);
  void uslCallback(const range_msg::ConstPtr& usl);
  void usfCallback(const range_msg::ConstPtr& usf);
  void hallCntCallback(const uint8_msg::ConstPtr& hallCnt);
  void hallDtCallback(const float64_msg::ConstPtr& hallDt);
  void hallDt8Callback(const float64_msg::ConstPtr& hallDt8);
  void driveBatteryCallback(const battery_msg::ConstPtr& vdBat);
  void systemBatteryCallback(const battery_msg::ConstPtr& vsBat);
  void cameraCallback(const image_msg::ConstPtr& img);
  void depthCallback(const image_msg::ConstPtr& img);

  Ui::Dashboard* ui;
  ros::NodeHandle* nh;
  int16_msg motorMessage;
  int16_msg steeringMessage;
  string_msg mode;

  ros::Publisher modeControl;
  ros::Publisher motorCommand;
  ros::Publisher steeringCommand;
  ros::Subscriber imuSub;
  ros::Subscriber magneticSub;
  ros::Subscriber usrSub;
  ros::Subscriber uslSub;
  ros::Subscriber usfSub;
  ros::Subscriber hallCntSub;
  ros::Subscriber hallDtSub;
  ros::Subscriber hallDt8Sub;
  ros::Subscriber vdBatSub;
  ros::Subscriber vsBatSub;
  ros::Subscriber cameraSub;
  ros::Subscriber depthSub;
  QTimer* timer;

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
