#include <pses_dashboard/dashboard.h>
Dashboard::Dashboard(ros::NodeHandle* nh, QWidget* parent)
    : QMainWindow(parent), ui(new Ui::Dashboard), nh(nh)
{
  ui->setupUi(this);
  fetchStartupParameters();
  reconfigureSpeedSlider();
  registerRosTopics();
  callGetFirmwareServide();
  callGetCarIdServide();
  callGetSidServide();
  configureVideoFeed();
  connectGuiSignals();

  ui->modeSelection->addItem(QString("Remote Control"), QVariant());
  ui->modeSelection->addItem(QString("Follow Wall"), QVariant());
  ui->modeSelection->addItem(QString("Roundtrip w. Obstacles"), QVariant());
  ui->modeSelection->addItem(QString("Park Car"), QVariant());
  ui->modeSelection->addItem(QString("Lane Detection"), QVariant());
  ui->modeSelection->addItem(QString("Exploration"), QVariant());

  initTopicPollingTimer();
}

Dashboard::~Dashboard() { delete ui; }

void Dashboard::fetchStartupParameters()
{
  if (nh->hasParam("pses_dashboard/mode_control_topic"))
  {
    nh->getParam("pses_dashboard/mode_control_topic", modeControlTopic);
  }
  else
  {
    modeControlTopic = DEFAULT_MODE_CONTROL_TOPIC;
  }
  if (nh->hasParam("pses_dashboard/steering_command_topic"))
  {
    nh->getParam("pses_dashboard/steering_command_topic", steeringCommandTopic);
  }
  else
  {
    steeringCommandTopic = DEFAULT_STEERING_COMMAND_TOPIC;
  }
  if (nh->hasParam("pses_dashboard/motor_command_topic"))
  {
    nh->getParam("pses_dashboard/motor_command_topic", motorCommandTopic);
  }
  else
  {
    motorCommandTopic = DEFAULT_MOTOR_COMMAND_TOPIC;
  }
  if (nh->hasParam("pses_dashboard/imu_topic"))
  {
    nh->getParam("pses_dashboard/imu_topic", imuTopic);
  }
  else
  {
    imuTopic = DEFAULT_IMU_TOPIC;
  }
  if (nh->hasParam("pses_dashboard/magnetic_topic"))
  {
    nh->getParam("pses_dashboard/magnetic_topic", magneticTopic);
  }
  else
  {
    magneticTopic = DEFAULT_MAGNETIC_TOPIC;
  }
  if (nh->hasParam("pses_dashboard/usr_topic"))
  {
    nh->getParam("pses_dashboard/usr_topic", usrTopic);
  }
  else
  {
    usrTopic = DEFAULT_USR_TOPIC;
  }
  if (nh->hasParam("pses_dashboard/usl_topic"))
  {
    nh->getParam("pses_dashboard/usl_topic", uslTopic);
  }
  else
  {
    uslTopic = DEFAULT_USL_TOPIC;
  }
  if (nh->hasParam("pses_dashboard/usf_topic"))
  {
    nh->getParam("pses_dashboard/usf_topic", usfTopic);
  }
  else
  {
    usfTopic = DEFAULT_USF_TOPIC;
  }
  if (nh->hasParam("pses_dashboard/hallcnt_topic"))
  {
    nh->getParam("pses_dashboard/hallcnt_topic", hallCntTopic);
  }
  else
  {
    hallCntTopic = DEFAULT_HALLCNT_TOPIC;
  }
  if (nh->hasParam("pses_dashboard/halldt_topic"))
  {
    nh->getParam("pses_dashboard/halldt_topic", hallDtTopic);
  }
  else
  {
    hallDtTopic = DEFAULT_HALLDT_TOPIC;
  }
  if (nh->hasParam("pses_dashboard/halldt8_topic"))
  {
    nh->getParam("pses_dashboard/halldt8_topic", hallDt8Topic);
  }
  else
  {
    hallDt8Topic = DEFAULT_HALLDT8_TOPIC;
  }
  if (nh->hasParam("pses_dashboard/sys_bat_topic"))
  {
    nh->getParam("pses_dashboard/sys_bat_topic", vsBatTopic);
  }
  else
  {
    vsBatTopic = DEFAULT_VSBAT_TOPIC;
  }
  if (nh->hasParam("pses_dashboard/drv_bat_topic"))
  {
    nh->getParam("pses_dashboard/drv_bat_topic", vdBatTopic);
  }
  else
  {
    vdBatTopic = DEFAULT_VDBAT_TOPIC;
  }
  if (nh->hasParam("pses_dashboard/get_car_id_service"))
  {
    nh->getParam("pses_dashboard/get_car_id_service", getCarIdService);
  }
  else
  {
    getCarIdService = DEFAULT_GET_CARID_SERVICE;
  }
  if (nh->hasParam("pses_dashboard/get_firmware_service"))
  {
    nh->getParam("pses_dashboard/get_firmware_service", getFirmwareService);
  }
  else
  {
    getFirmwareService = DEFAULT_GET_FIRMWARE_SERVICE;
  }
  if (nh->hasParam("pses_dashboard/get_sid_service"))
  {
    nh->getParam("pses_dashboard/get_sid_service", getSidService);
  }
  else
  {
    getSidService = DEFAULT_GET_SID_SERVICE;
  }
  if (nh->hasParam("pses_dashboard/image_color_topic"))
  {
    nh->getParam("pses_dashboard/image_color_topic", imageColorTopic);
  }
  else
  {
    imageColorTopic = DEFAULT_IMAGE_COLOR_TOPIC;
  }
  if (nh->hasParam("pses_dashboard/image_depth_topic"))
  {
    nh->getParam("pses_dashboard/image_depth_topic", imageDepthTopic);
  }
  else
  {
    imageDepthTopic = DEFAULT_IMAGE_DEPTH_TOPIC;
  }
  if (nh->hasParam("pses_dashboard/toggle_kinect_service"))
  {
    nh->getParam("pses_dashboard/toggle_kinect_service", toggleKinectService);
  }
  else
  {
    toggleKinectService = DEFAULT_TOGGLE_KINECT_SERVICE;
  }
  if (nh->hasParam("pses_dashboard/toggle_motor_service"))
  {
    nh->getParam("pses_dashboard/toggle_motor_service", toggleMotorService);
  }
  else
  {
    toggleMotorService = DEFAULT_TOGGLE_MOTOR_SERVICE;
  }
  if (nh->hasParam("pses_dashboard/toggle_us_service"))
  {
    nh->getParam("pses_dashboard/toggle_us_service", toggleUSService);
  }
  else
  {
    toggleUSService = DEFAULT_TOGGLE_US_SERVICE;
  }
  if (nh->hasParam("pses_dashboard/toggle_daq_service"))
  {
    nh->getParam("pses_dashboard/toggle_daq_service", toggleDAQService);
  }
  else
  {
    toggleDAQService = DEFAULT_TOGGLE_DAQ_SERVICE;
  }
  if (nh->hasParam("pses_dashboard/video_feed"))
  {
    nh->getParam("pses_dashboard/video_feed", videoFeedMode);
  }
  else
  {
    videoFeedMode = DEFAULT_VIDEO_FEED_MODE;
  }
  if (nh->hasParam("pses_dashboard/max_fwd_speed"))
  {
    nh->getParam("pses_dashboard/max_fwd_speed", maxForwardSpeed);
  }
  else
  {
    maxForwardSpeed = DEFAULT_MAX_FWD_SPEED;
  }
  if (nh->hasParam("pses_dashboard/max_bwd_speed"))
  {
    nh->getParam("pses_dashboard/max_bwd_speed", maxReverseSpeed);
  }
  else
  {
    maxReverseSpeed = DEFAULT_MAX_BWD_SPEED;
  }
  if (nh->hasParam("pses_dashboard/max_left_steer"))
  {
    nh->getParam("pses_dashboard/max_left_steer", maxLeftSteering);
  }
  else
  {
    maxLeftSteering = DEFAULT_MAX_LEFT_STEER;
  }
  if (nh->hasParam("pses_dashboard/max_right_steer"))
  {
    nh->getParam("pses_dashboard/max_right_steer", maxRightSteering);
  }
  else
  {
    maxRightSteering = DEFAULT_MAX_RIGHT_STEER;
  }
  if (nh->hasParam("pses_dashboard/steering_step"))
  {
    nh->getParam("pses_dashboard/steering_step", steeringStep);
  }
  else
  {
    steeringStep = DEFAULT_STEER_STEP;
  }
  if (nh->hasParam("pses_dashboard/speed_step"))
  {
    nh->getParam("pses_dashboard/speed_step", speedStep);
  }
  else
  {
    speedStep = DEFAULT_SPEED_STEP;
  }
  leftSgn = boost::math::sign(maxLeftSteering);
  rightSgn = boost::math::sign(maxRightSteering);
  fwdSgn = boost::math::sign(maxForwardSpeed);
  bwdSgn = boost::math::sign(maxReverseSpeed);
}

void Dashboard::reconfigureSpeedSlider() {
  ui->speedSlider->setMaximum(maxForwardSpeed);
  ui->speedSlider->setMinimum(maxReverseSpeed);
  ui->speedSlider->setSingleStep(speedStep);
  ui->speedSlider->setSliderPosition(0);
  ui->speedSlider->setValue(0);
  ui->maxSpeed->setText(QString(std::to_string(maxForwardSpeed).c_str()));
  ui->minSpeed->setText(QString(std::to_string(maxReverseSpeed).c_str()));
  ui->steeringSlider->setMaximum(maxRightSteering);
  ui->steeringSlider->setMinimum(maxLeftSteering);
  ui->steeringSlider->setSingleStep(steeringStep);
  ui->steeringSlider->setSliderPosition(0);
  ui->steeringSlider->setValue(0);
  ui->maxRightSteering->setText(QString(std::to_string(maxRightSteering).c_str()));
  ui->maxLeftSteering->setText(QString(std::to_string(maxLeftSteering).c_str()));
}

void Dashboard::registerRosTopics()
{
  modeControl = nh->advertise<string_msg>(modeControlTopic, 10);
  steeringCommand = nh->advertise<int16_msg>(steeringCommandTopic, 1);
  motorCommand = nh->advertise<int16_msg>(motorCommandTopic, 1);
  imuSub = nh->subscribe<imu_msg>(
      imuTopic, 10, boost::bind(&Dashboard::imuCallback, this, _1));
  magneticSub = nh->subscribe<magnetic_msg>(
      magneticTopic, 10, boost::bind(&Dashboard::magneticCallback, this, _1));
  usrSub = nh->subscribe<range_msg>(
      usrTopic, 10, boost::bind(&Dashboard::usrCallback, this, _1));
  uslSub = nh->subscribe<range_msg>(
      uslTopic, 10, boost::bind(&Dashboard::uslCallback, this, _1));
  usfSub = nh->subscribe<range_msg>(
      usfTopic, 10, boost::bind(&Dashboard::usfCallback, this, _1));
  hallCntSub = nh->subscribe<uint8_msg>(
      hallCntTopic, 10, boost::bind(&Dashboard::hallCntCallback, this, _1));
  hallDtSub = nh->subscribe<float64_msg>(
      hallDtTopic, 10, boost::bind(&Dashboard::hallDtCallback, this, _1));
  hallDt8Sub = nh->subscribe<float64_msg>(
      hallDt8Topic, 10, boost::bind(&Dashboard::hallDt8Callback, this, _1));
  vdBatSub = nh->subscribe<battery_msg>(
      vdBatTopic, 10, boost::bind(&Dashboard::driveBatteryCallback, this, _1));
  vsBatSub = nh->subscribe<battery_msg>(
      vsBatTopic, 10, boost::bind(&Dashboard::systemBatteryCallback, this, _1));
}

void Dashboard::callGetFirmwareServide()
{
  if (!ros::service::waitForService(getFirmwareService, 500))
    ROS_WARN_STREAM("Get car id service not availible!");
  pses_ucbridge::GetFirmwareVersion::Request firmwareRequest;
  pses_ucbridge::GetFirmwareVersion::Response firmwareResponse;
  ros::service::call(getFirmwareService, firmwareRequest, firmwareResponse);
  if (firmwareResponse.answer_received)
  {
    ui->firmware_label->setText(QString(firmwareResponse.version.c_str()));
  }
  else
  {
    ui->firmware_label->setText(QString("N/A"));
  }
}

void Dashboard::callGetCarIdServide()
{
  if (!ros::service::waitForService(getCarIdService, 500))
    ROS_WARN_STREAM("Get car id service not availible!");
  pses_ucbridge::GetControllerID::Request idRequest;
  pses_ucbridge::GetControllerID::Response idResponse;
  ros::service::call(getCarIdService, idRequest, idResponse);
  if (idResponse.answer_received)
  {
    ui->id_label->setText(QString(std::to_string(idResponse.ID).c_str()));
  }
  else
  {
    ui->id_label->setText(QString("N/A"));
  }
}

void Dashboard::callGetSidServide()
{
  if (!ros::service::waitForService(getSidService, 500))
    ROS_WARN_STREAM("Get car sid service not availible!");
  pses_ucbridge::GetSessionID::Request sidRequest;
  pses_ucbridge::GetSessionID::Response sidResponse;
  ros::service::call(getSidService, sidRequest, sidResponse);
  if (sidResponse.answer_received)
  {
    ui->sid_label->setText(QString(std::to_string(sidResponse.SID).c_str()));
  }
  else
  {
    ui->sid_label->setText(QString("N/A"));
  }
}

void Dashboard::configureVideoFeed()
{
  QPixmap videoFeed(1280, 720);
  ui->display_camera->setPixmap(videoFeed);
  ui->camera_selection->addItem(QString(VIDEO_FEED_MODE_COLOR.c_str()),
                                QVariant());
  ui->camera_selection->addItem(QString(VIDEO_FEED_MODE_DEPTH.c_str()),
                                QVariant());
  ui->camera_selection->addItem(QString(DEFAULT_VIDEO_FEED_MODE.c_str()),
                                QVariant());

  if (videoFeedMode.compare(VIDEO_FEED_MODE_COLOR) == 0)
  {
    cameraSub = nh->subscribe<image_msg>(
        imageColorTopic, 10, boost::bind(&Dashboard::cameraCallback, this, _1));
    ui->camera_selection->setCurrentIndex(0);
  }
  else if (videoFeedMode.compare(VIDEO_FEED_MODE_DEPTH) == 0)
  {
    depthSub = nh->subscribe<image_msg>(
        imageDepthTopic, 10, boost::bind(&Dashboard::depthCallback, this, _1));
    ui->camera_selection->setCurrentIndex(1);
  }
  else if (videoFeedMode.compare(DEFAULT_VIDEO_FEED_MODE) == 0)
  {
    ui->camera_selection->setCurrentIndex(2);
  }
}

void Dashboard::connectGuiSignals()
{
  connect(ui->speedSlider, SIGNAL(valueChanged(int)), this,
          SLOT(valueChangedSpeed(int)));
  connect(ui->steeringSlider, SIGNAL(valueChanged(int)), this,
          SLOT(valueChangedSteering(int)));
  connect(ui->maxSpeed, SIGNAL(clicked()), this, SLOT(maxSpeedClicked()));
  connect(ui->minSpeed, SIGNAL(clicked()), this, SLOT(minSpeedClicked()));
  connect(ui->zeroSpeed, SIGNAL(clicked()), this, SLOT(zeroSpeedClicked()));
  connect(ui->maxRightSteering, SIGNAL(clicked()), this, SLOT(maxRightSteeringClicked()));
  connect(ui->maxLeftSteering, SIGNAL(clicked()), this, SLOT(maxLeftSteeringClicked()));
  connect(ui->centerSteering, SIGNAL(clicked()), this,
          SLOT(centerSteeringClicked()));
  connect(ui->kinectToggle, SIGNAL(clicked()), this, SLOT(toggleKinect()));
  connect(ui->usToggle, SIGNAL(clicked()), this, SLOT(toggleUS()));
  connect(ui->motorToggle, SIGNAL(clicked()), this, SLOT(toggleMotor()));
  connect(ui->daqToggle, SIGNAL(clicked()), this, SLOT(toggleDAQ()));
  connect(ui->camera_selection, SIGNAL(currentIndexChanged(int)), this,
          SLOT(cameraSelect(int)));
  connect(ui->modeSelection, SIGNAL(currentIndexChanged(int)), this,
          SLOT(modeSelect(int)));
}

void Dashboard::initTopicPollingTimer()
{
  timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(pollNodeHandle()));
  timer->start(100);
}

void Dashboard::imuCallback(const imu_msg::ConstPtr& imu)
{
  ui->sensor_ax->display(imu->linear_acceleration.x);
  ui->sensor_ay->display(imu->linear_acceleration.y);
  ui->sensor_az->display(imu->linear_acceleration.z);
  ui->sensor_wx->display(imu->angular_velocity.x);
  ui->sensor_wy->display(imu->angular_velocity.y);
  ui->sensor_wz->display(imu->angular_velocity.z);
}

void Dashboard::magneticCallback(const magnetic_msg::ConstPtr& magnetic)
{
  ui->sensor_mx->display(magnetic->magnetic_field.x * 1000);
  ui->sensor_my->display(magnetic->magnetic_field.y * 1000);
  ui->sensor_mz->display(magnetic->magnetic_field.z * 1000);
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

void Dashboard::hallCntCallback(const uint8_msg::ConstPtr& hallCnt)
{
  ui->sensor_hall_count->display(hallCnt->data);
}

void Dashboard::hallDtCallback(const float64_msg::ConstPtr& hallDt)
{
  ui->sensor_hall_dt->display(hallDt->data);
}

void Dashboard::hallDt8Callback(const float64_msg::ConstPtr& hallDt8)
{
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
  int speedSgn;
  int steeringSgn;

  switch (event->key())
  {
  case Qt::Key_W:
  {
    speed = speed + fwdSgn * speedStep;
    speedSgn = boost::math::sign(speed);
    if (std::abs(speed) <= std::abs(maxForwardSpeed) || speedSgn != fwdSgn)
    {
      motorMessage.data = speed;
      motorCommand.publish(motorMessage);
    }
    break;
  }

  case Qt::Key_S:
  {
    speed = speed + bwdSgn * speedStep;
    speedSgn = boost::math::sign(speed);
    if (std::abs(speed) <= std::abs(maxReverseSpeed) || speedSgn != bwdSgn)
    {
      motorMessage.data = speed;
      motorCommand.publish(motorMessage);
    }
    break;
  }

  case Qt::Key_A:
  {
    steering = steering + leftSgn * steeringStep;
    steeringSgn = boost::math::sign(steering);
    if (std::abs(steering) <= std::abs(maxLeftSteering) || steeringSgn != leftSgn)
    {
      steeringMessage.data = steering;
      steeringCommand.publish(steeringMessage);
    }
    break;
  }

  case Qt::Key_D:
  {
    steering = steering + rightSgn * steeringStep;
    steeringSgn = boost::math::sign(steering);
    if (std::abs(steering) <= std::abs(maxRightSteering) || steeringSgn != rightSgn)
    {
      steeringMessage.data = steering;
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
  pses_ucbridge::ToggleKinect::Request kinectRequest;
  kinectRequest.kinect_on = ui->kinectToggle->isChecked();
  pses_ucbridge::ToggleKinect::Response kinectResponse;
  if (!ros::service::waitForService(toggleKinectService, 500))
    ROS_WARN_STREAM("Toggle kinect service not availible!");
  ros::service::call(toggleKinectService, kinectRequest, kinectResponse);
  if (!kinectResponse.was_set)
    ui->kinectToggle->setChecked(false);
}

void Dashboard::toggleUS()
{
  pses_ucbridge::ToggleUS::Request usRequest;
  usRequest.us_on = ui->usToggle->isChecked();
  pses_ucbridge::ToggleUS::Response usResponse;
  if (!ros::service::waitForService(toggleUSService, 500))
    ROS_WARN_STREAM("Toggle us service not availible!");
  ros::service::call(toggleUSService, usRequest, usResponse);
  if (!usResponse.was_set)
    ui->usToggle->setChecked(false);
}

void Dashboard::toggleMotor()
{
  pses_ucbridge::ToggleMotor::Request motorRequest;
  motorRequest.motor_on = ui->motorToggle->isChecked();
  pses_ucbridge::ToggleMotor::Response motorResponse;
  if (!ros::service::waitForService(toggleMotorService, 500))
    ROS_WARN_STREAM("Toggle motor service not availible!");
  ros::service::call(toggleMotorService, motorRequest, motorResponse);
  if (!motorResponse.was_set)
    ui->motorToggle->setChecked(false);
}

void Dashboard::toggleDAQ()
{
  pses_ucbridge::ToggleDAQ::Request daqRequest;
  daqRequest.DAQ_on = ui->daqToggle->isChecked();
  pses_ucbridge::ToggleDAQ::Response daqResponse;
  if (!ros::service::waitForService(toggleDAQService, 500))
    ROS_WARN_STREAM("Toggle daq service not availible!");
  ros::service::call(toggleDAQService, daqRequest, daqResponse);
  if (!daqResponse.was_set)
    ui->daqToggle->setChecked(false);
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
    cameraSub = nh->subscribe<image_msg>(
        imageColorTopic, 10, boost::bind(&Dashboard::cameraCallback, this, _1));
  }
  else if (index == 1)
  {
    cameraSub.shutdown();
    depthSub = nh->subscribe<image_msg>(
        imageDepthTopic, 10, boost::bind(&Dashboard::depthCallback, this, _1));
  }
  else
  {
    depthSub.shutdown();
    cameraSub.shutdown();
  }
}

void Dashboard::valueChangedSpeed(int value)
{
  motorMessage.data = value * speedStep;
  motorCommand.publish(motorMessage);
  ros::spinOnce();
}

void Dashboard::valueChangedSteering(int value)
{
  steeringMessage.data = value * steeringStep;
  steeringCommand.publish(steeringMessage);
  ros::spinOnce();
}

void Dashboard::maxSpeedClicked()
{
  motorMessage.data = maxForwardSpeed;
  motorCommand.publish(motorMessage);
  ui->speedSlider->blockSignals(true);
  ui->speedSlider->setValue(maxForwardSpeed);
  ui->speedSlider->blockSignals(false);
  ros::spinOnce();
}

void Dashboard::minSpeedClicked()
{
  motorMessage.data = maxReverseSpeed;
  motorCommand.publish(motorMessage);
  ui->speedSlider->blockSignals(true);
  ui->speedSlider->setValue(maxReverseSpeed);
  ui->speedSlider->blockSignals(false);
  ros::spinOnce();
}

void Dashboard::zeroSpeedClicked()
{
  motorMessage.data = 0;
  motorCommand.publish(motorMessage);
  ui->speedSlider->blockSignals(true);
  ui->speedSlider->setValue(0);
  ui->speedSlider->blockSignals(false);
  ros::spinOnce();
}

void Dashboard::maxLeftSteeringClicked()
{
  steeringMessage.data = maxLeftSteering;
  steeringCommand.publish(steeringMessage);
  ui->steeringSlider->blockSignals(true);
  ui->steeringSlider->setValue(maxLeftSteering);
  ui->steeringSlider->blockSignals(false);
  ros::spinOnce();
}

void Dashboard::maxRightSteeringClicked()
{
  steeringMessage.data = maxRightSteering;
  steeringCommand.publish(steeringMessage);
  ui->steeringSlider->blockSignals(true);
  ui->steeringSlider->setValue(maxRightSteering);
  ui->steeringSlider->blockSignals(false);
  ros::spinOnce();
}

void Dashboard::centerSteeringClicked()
{
  steeringMessage.data = 0;
  steeringCommand.publish(steeringMessage);
  ui->steeringSlider->blockSignals(true);
  ui->steeringSlider->setValue(0);
  ui->steeringSlider->blockSignals(false);
  ros::spinOnce();
}
