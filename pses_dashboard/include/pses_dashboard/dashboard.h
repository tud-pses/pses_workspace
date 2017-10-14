#ifndef DASHBOARD_H
#define DASHBOARD_H

#include <math.h>
#include <string>
#include <sstream>
#include <boost/math/special_functions/sign.hpp>

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

#include <pses_ucbridge/GetFirmwareVersion.h>
#include <pses_ucbridge/GetControllerID.h>
#include <pses_ucbridge/GetSessionID.h>
#include <pses_ucbridge/ToggleKinect.h>
#include <pses_ucbridge/ToggleMotor.h>
#include <pses_ucbridge/ToggleUS.h>
#include <pses_ucbridge/ToggleDAQ.h>

namespace Dash
{
typedef std_msgs::String string_msg;
typedef sensor_msgs::Image image_msg;
typedef sensor_msgs::Imu imu_msg;
typedef sensor_msgs::BatteryState battery_msg;
typedef sensor_msgs::MagneticField magnetic_msg;
typedef sensor_msgs::Range range_msg;
typedef std_msgs::Int16 int16_msg;
typedef std_msgs::Float64 float64_msg;
typedef std_msgs::UInt8 uint8_msg;

static const std::string DEFAULT_MODE_CONTROL_TOPIC =
    "/dasboard/mode_control";
static const std::string DEFAULT_STEERING_COMMAND_TOPIC =
    "/uc_bridge/set_steering_level_msg";
static const std::string DEFAULT_MOTOR_COMMAND_TOPIC =
    "/uc_bridge/set_motor_level_msg";
static const std::string DEFAULT_IMU_TOPIC = "/uc_bridge/imu";
static const std::string DEFAULT_MAGNETIC_TOPIC = "/uc_bridge/mag";
static const std::string DEFAULT_USR_TOPIC = "/uc_bridge/usr";
static const std::string DEFAULT_USF_TOPIC = "/uc_bridge/usf";
static const std::string DEFAULT_USL_TOPIC = "/uc_bridge/usl";
static const std::string DEFAULT_HALLCNT_TOPIC = "/uc_bridge/hall_cnt";
static const std::string DEFAULT_HALLDT_TOPIC = "/uc_bridge/hall_dt";
static const std::string DEFAULT_HALLDT8_TOPIC = "/uc_bridge/hall_dt8";
static const std::string DEFAULT_VDBAT_TOPIC = "/uc_bridge/vdbat";
static const std::string DEFAULT_VSBAT_TOPIC = "/uc_bridge/vsbat";
static const std::string DEFAULT_GET_FIRMWARE_SERVICE =
    "/uc_bridge/get_firmware_version";
static const std::string DEFAULT_GET_CARID_SERVICE =
    "/uc_bridge/get_controller_id";
static const std::string DEFAULT_GET_SID_SERVICE = "/uc_bridge/get_session_id";
static const std::string DEFAULT_IMAGE_COLOR_TOPIC = "kinect2/qhd/image_color";
static const std::string DEFAULT_IMAGE_DEPTH_TOPIC = "kinect2/sd/image_depth";
static const std::string DEFAULT_TOGGLE_KINECT_SERVICE =
    "/uc_bridge/toggle_kinect";
static const std::string DEFAULT_TOGGLE_MOTOR_SERVICE =
    "/uc_bridge/toggle_motor";
static const std::string DEFAULT_TOGGLE_US_SERVICE = "/uc_bridge/toggle_us";
static const std::string DEFAULT_TOGGLE_DAQ_SERVICE = "/uc_bridge/toggle_daq";
static const std::string DEFAULT_VIDEO_FEED_MODE = "Off";
static const int DEFAULT_MAX_FWD_SPEED = 1000;
static const int DEFAULT_MAX_BWD_SPEED = -500;
static const int DEFAULT_SPEED_STEP = 50;
static const int DEFAULT_MAX_LEFT_STEER = -1000;
static const int DEFAULT_MAX_RIGHT_STEER = 1000;
static const int DEFAULT_STEER_STEP = 100;
static const std::string VIDEO_FEED_MODE_COLOR = "Color Image (1280x720)";
static const std::string VIDEO_FEED_MODE_DEPTH = "Depth Image (640x480)";
}

using namespace Dash;

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
  void fetchStartupParameters();
  void reconfigureSpeedSlider();
  void registerRosTopics();
  void callGetFirmwareServide();
  void callGetCarIdServide();
  void callGetSidServide();
  void configureVideoFeed();
  void connectGuiSignals();
  void initTopicPollingTimer();
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
  std::string modeControlTopic, steeringCommandTopic, motorCommandTopic,
      imuTopic, magneticTopic, usrTopic, uslTopic, usfTopic, hallCntTopic,
      hallDtTopic, hallDt8Topic, vdBatTopic, vsBatTopic, getFirmwareService,
      getCarIdService, imageColorTopic, imageDepthTopic, toggleKinectService,
      getSidService, toggleUSService, toggleMotorService, toggleDAQService;
  int maxForwardSpeed, maxReverseSpeed, speedStep, maxLeftSteering,
      maxRightSteering, steeringStep;
  int leftSgn, rightSgn, fwdSgn, bwdSgn;
  std::string videoFeedMode;
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
  void toggleUS();
  void toggleMotor();
  void toggleDAQ();
  void modeSelect(int index);
  void cameraSelect(int index);
  void valueChangedSpeed(int value);
  void valueChangedSteering(int value);
  void maxSpeedClicked();
  void minSpeedClicked();
  void zeroSpeedClicked();
  void maxRightSteeringClicked();
  void maxLeftSteeringClicked();
  void centerSteeringClicked();
  void pollNodeHandle();
};

#endif // DASHBOARD_H
