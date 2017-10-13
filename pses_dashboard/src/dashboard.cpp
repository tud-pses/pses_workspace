#include <pses_dashboard/dashboard.h>
Dashboard::Dashboard(ros::NodeHandle* nh, QWidget* parent)
    : QMainWindow(parent), ui(new Ui::Dashboard), nh(nh)
{
  ui->setupUi(this);
  modeControl = nh->advertise<string_msg>("pses_basis/mode_control", 10);
  steeringCommand =
      nh->advertise<int16_msg>("/uc_bridge/set_steering_level_msg", 10);
  motorCommand = nh->advertise<int16_msg>("/uc_bridge/set_motor_level_msg", 10);
  imuSub = nh->subscribe<imu_msg>(
      "/uc_bridge/imu", 10, boost::bind(&Dashboard::imuCallback, this, _1));
  magneticSub = nh->subscribe<magnetic_msg>(
      "/uc_bridge/mag", 10,
      boost::bind(&Dashboard::magneticCallback, this, _1));
  usrSub = nh->subscribe<range_msg>(
      "/uc_bridge/usr", 10, boost::bind(&Dashboard::usrCallback, this, _1));
  uslSub = nh->subscribe<range_msg>(
      "/uc_bridge/usl", 10, boost::bind(&Dashboard::uslCallback, this, _1));
  usfSub = nh->subscribe<range_msg>(
      "/uc_bridge/usf", 10, boost::bind(&Dashboard::usfCallback, this, _1));
  hallCntSub = nh->subscribe<uint8_msg>(
        "/uc_bridge/hall_cnt", 10, boost::bind(&Dashboard::hallCntCallback, this, _1));
  hallDtSub = nh->subscribe<float64_msg>(
        "/uc_bridge/hall_dt", 10, boost::bind(&Dashboard::hallDtCallback, this, _1));
  hallDt8Sub = nh->subscribe<float64_msg>(
        "/uc_bridge/hall_dt8", 10, boost::bind(&Dashboard::hallDt8Callback, this, _1));
  vdBatSub = nh->subscribe<battery_msg>(
      "/uc_bridge/vdbat", 10,
      boost::bind(&Dashboard::driveBatteryCallback, this, _1));
  vsBatSub = nh->subscribe<battery_msg>(
      "/uc_bridge/vsbat", 10,
      boost::bind(&Dashboard::systemBatteryCallback, this, _1));

  connect(ui->speedSlider, SIGNAL(valueChanged(int)), this,
          SLOT(valueChangedSpeed(int)));
  connect(ui->steeringSlider, SIGNAL(valueChanged(int)), this,
          SLOT(valueChangedSteering(int)));
  connect(ui->maxSpeed, SIGNAL(clicked()), this, SLOT(maxSpeedClicked()));
  connect(ui->minSpeed, SIGNAL(clicked()), this, SLOT(minSpeedClicked()));
  connect(ui->zeroSpeed, SIGNAL(clicked()), this, SLOT(zeroSpeedClicked()));
  connect(ui->maxSteering, SIGNAL(clicked()), this, SLOT(maxSteeringClicked()));
  connect(ui->minSteering, SIGNAL(clicked()), this, SLOT(minSteeringClicked()));
  connect(ui->centerSteering, SIGNAL(clicked()), this,
          SLOT(centerSteeringClicked()));
  connect(ui->kinectToggle, SIGNAL(clicked()), this, SLOT(toggleKinect()));

  ui->modeSelection->addItem(QString("Remote Control"), QVariant());
  ui->modeSelection->addItem(QString("Follow Wall"), QVariant());
  ui->modeSelection->addItem(QString("Roundtrip w. Obstacles"), QVariant());
  ui->modeSelection->addItem(QString("Park Car"), QVariant());
  ui->modeSelection->addItem(QString("Lane Detection"), QVariant());
  ui->modeSelection->addItem(QString("Exploration"), QVariant());
  connect(ui->modeSelection, SIGNAL(currentIndexChanged(int)), this,
          SLOT(modeSelect(int)));

  ros::service::waitForService("/uc_bridge/get_firmware_version");
  pses_basis::GetFirmwareVersion::Request firmwareRequest;
  pses_basis::GetFirmwareVersion::Response firmwareResponse;
  ros::service::call("/uc_bridge/get_firmware_version", firmwareRequest,
                     firmwareResponse);
  if (firmwareResponse.answer_received)
  {
    ui->firmware_label->setText(QString(firmwareResponse.version.c_str()));
  }
  else
  {
    ui->firmware_label->setText(QString("N/A"));
  }
  pses_basis::GetControllerID::Request idRequest;
  pses_basis::GetControllerID::Response idResponse;
  ros::service::call("/uc_bridge/get_controller_id", idRequest, idResponse);
  if (idResponse.answer_received)
  {
    ui->id_label->setText(QString(std::to_string(idResponse.ID).c_str()));
  }
  else
  {
    ui->id_label->setText(QString("N/A"));
  }

  std::string videoMode = "off";
  nh->getParam("car_dashboard/video_feed", videoMode);
  QPixmap videoFeed(1280, 720);
  ui->display_camera->setPixmap(videoFeed);
  ui->camera_selection->addItem(QString("Color Image (1280x720)"), QVariant());
  ui->camera_selection->addItem(QString("Depth Image (640x480) "), QVariant());
  ui->camera_selection->addItem(QString("Off"), QVariant());
  connect(ui->camera_selection, SIGNAL(currentIndexChanged(int)), this,
          SLOT(cameraSelect(int)));

  if (videoMode.compare("image_color") == 0)
  {
    cameraSub = nh->subscribe<image_msg>(
        "kinect2/qhd/image_color", 10,
        boost::bind(&Dashboard::cameraCallback, this, _1));
    ui->camera_selection->setCurrentIndex(0);
  }
  else if (videoMode.compare("image_depth") == 0)
  {
    depthSub = nh->subscribe<image_msg>(
        "kinect2/sd/image_depth", 10,
        boost::bind(&Dashboard::depthCallback, this, _1));
    ui->camera_selection->setCurrentIndex(1);
  }
  else if (videoMode.compare("off") == 0)
  {
    ui->camera_selection->setCurrentIndex(2);
  }

  timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(pollNodeHandle()));
  timer->start(100);
}

Dashboard::~Dashboard() { delete ui; }

void Dashboard::imuCallback(const imu_msg::ConstPtr& imu)
{
  ui->sensor_ax->display(imu->linear_acceleration.x);
  ui->sensor_ay->display(imu->linear_acceleration.y);
  ui->sensor_az->display(imu->linear_acceleration.z);
  ui->sensor_wx->display(imu->angular_velocity.x);
  ui->sensor_wy->display(imu->angular_velocity.y);
  ui->sensor_wz->display(imu->angular_velocity.z);
}

void Dashboard::magneticCallback(const magnetic_msg::ConstPtr& magnetic) {
  ui->sensor_mx->display(magnetic->magnetic_field.x*1000);
  ui->sensor_my->display(magnetic->magnetic_field.y*1000);
  ui->sensor_mz->display(magnetic->magnetic_field.z*1000);
}

void Dashboard::usrCallback(const range_msg::ConstPtr& usr)
{
  ui->sensor_usr->display(usr->range);
}

void Dashboard::uslCallback(const range_msg::ConstPtr& usl)
{
  ui->sensor_usl->display(usl->range);
}

void Dashboard::usfCallback(const range_msg::ConstPtr& usf)
{
  ui->sensor_usf->display(usf->range);
}

void Dashboard::hallCntCallback(const uint8_msg::ConstPtr& hallCnt){
  ui->sensor_hall_count->display(hallCnt->data);
}

void Dashboard::hallDtCallback(const float64_msg::ConstPtr& hallDt){
  ui->sensor_hall_dt->display(hallDt->data);
}

void Dashboard::hallDt8Callback(const float64_msg::ConstPtr& hallDt8){
  ui->sensor_hall_dtf->display(hallDt8->data);
}

void Dashboard::driveBatteryCallback(const battery_msg::ConstPtr& vdBat)
{
  ui->sensor_batt_motor->display(vdBat->voltage);
}

void Dashboard::systemBatteryCallback(const battery_msg::ConstPtr& vsBat)
{
  ui->sensor_batt_sys->display(vsBat->voltage);
}

void Dashboard::cameraCallback(const image_msg::ConstPtr& img)
{
  QImage image(&img->data[0], (int)img->width, (int)img->height,
               QImage::Format_RGB888);
  ui->display_camera->setPixmap(QPixmap::fromImage(image.rgbSwapped()));
}

void Dashboard::depthCallback(const image_msg::ConstPtr& img)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(*img, img->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  cv::Mat im16 = cv_ptr->image;
  double min, max;
  cv::minMaxIdx(im16, &min, &max);
  cv::Mat im8;
  im16.convertTo(im8, CV_8UC1, 255 / (max - min), -min);
  QImage image(im8.data, im8.cols, im8.rows, static_cast<int>(im8.step),
               QImage::Format_Indexed8);
  ui->display_camera->setPixmap(QPixmap::fromImage(image));
}

void Dashboard::keyPressEvent(QKeyEvent* event)
{
  int speed = motorMessage.data;       // cmd.motor_level;
  int steering = steeringMessage.data; // cmd.steering_level;

  switch (event->key())
  {
  case Qt::Key_W:
  {
    if (speed < 1000)
    {
      motorMessage.data = speed + 50;
      motorCommand.publish(motorMessage);
    }
    break;
  }

  case Qt::Key_S:
  {
    if (speed > -500)
    {
      motorMessage.data = speed - 50;
      motorCommand.publish(motorMessage);
    }
    break;
  }

  case Qt::Key_A:
  {
    if (steering < 1000)
    {
      steeringMessage.data = steering + 100;
      steeringCommand.publish(steeringMessage);
    }
    break;
  }

  case Qt::Key_D:
  {
    if (steering > -1000)
    {
      steeringMessage.data = steering - 100;
      steeringCommand.publish(steeringMessage);
    }
    break;
  }

  case Qt::Key_Space:
  {
    motorMessage.data = 0;
    motorCommand.publish(motorMessage);
    break;
  }
  }
  ui->speedSlider->blockSignals(true);
  ui->steeringSlider->blockSignals(true);
  ui->speedSlider->setValue(motorMessage.data);
  ui->steeringSlider->setValue(steeringMessage.data);
  ui->speedSlider->blockSignals(false);
  ui->steeringSlider->blockSignals(false);
  ros::spinOnce();
}

void Dashboard::pollNodeHandle()
{
  ros::spinOnce();
  timer->start(100);
}

void Dashboard::toggleKinect()
{
  pses_basis::ToggleKinect::Request kinectRequest;
  kinectRequest.kinect_on = ui->kinectToggle->isChecked();
  pses_basis::ToggleKinect::Response kinectResponse;
  ros::service::waitForService("/uc_bridge/toggle_kinect");
  ros::service::call("/uc_bridge/toggle_kinect", kinectRequest, kinectResponse);
  if (!kinectResponse.was_set)
    ui->kinectToggle->setChecked(false);
}

void Dashboard::modeSelect(int index)
{
  mode.data = ui->modeSelection->itemText(index).toStdString();
  modeControl.publish(mode);
  ros::spinOnce();
}

void Dashboard::cameraSelect(int index)
{
  if (index == 0)
  {
    depthSub.shutdown();
    cameraSub = nh->subscribe<image_msg>("kinect2/qhd/image_color", 10,
                                         boost::bind(&Dashboard::cameraCallback, this, _1));
  }
  else if (index == 1)
  {
    cameraSub.shutdown();
    depthSub = nh->subscribe<image_msg>("kinect2/sd/image_depth", 10,
                                        boost::bind(&Dashboard::depthCallback, this, _1));
  }
  else
  {
    depthSub.shutdown();
    cameraSub.shutdown();
  }
}

void Dashboard::valueChangedSpeed(int value)
{
  motorMessage.data = value * 50;
  motorCommand.publish(motorMessage);
  ros::spinOnce();
}

void Dashboard::valueChangedSteering(int value)
{
  steeringMessage.data = value * 20;
  steeringCommand.publish(steeringMessage);
  ros::spinOnce();
}

void Dashboard::maxSpeedClicked()
{
  motorMessage.data = 1000;
  motorCommand.publish(motorMessage);
  ui->speedSlider->setValue(20);
  ros::spinOnce();
}

void Dashboard::minSpeedClicked()
{
  motorMessage.data = -1000;
  motorCommand.publish(motorMessage);
  ui->speedSlider->setValue(-20);
  ros::spinOnce();
}

void Dashboard::zeroSpeedClicked()
{
  motorMessage.data = 0;
  motorCommand.publish(motorMessage);
  ui->speedSlider->setValue(0);
  ros::spinOnce();
}

void Dashboard::maxSteeringClicked()
{
  steeringMessage.data = 1000;
  steeringCommand.publish(steeringMessage);
  ui->steeringSlider->setValue(50);
  ros::spinOnce();
}

void Dashboard::minSteeringClicked()
{
  steeringMessage.data = 1000;
  steeringCommand.publish(steeringMessage);
  ui->steeringSlider->setValue(-50);
  ros::spinOnce();
}

void Dashboard::centerSteeringClicked()
{
  steeringMessage.data = 0;
  steeringCommand.publish(steeringMessage);
  ui->steeringSlider->setValue(0);
  ros::spinOnce();
}
