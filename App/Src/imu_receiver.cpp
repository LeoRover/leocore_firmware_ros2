#include "app/imu_receiver.hpp"

static constexpr float PI = 3.141592653F;
static constexpr float GRAVITATIONAL_ACCELERATION = 9.80665F;
static constexpr float DEGREE_TO_RADIAN = 2.0F * PI / 360.0F;
static constexpr float TEMP_RESOLUTION = 1.0F / 132.48F;
static constexpr float TEMP_OFFSET = 25.0F;

void ImuReceiver::init() {
  icm_.init(AFS_2G, GFS_250DPS, AODR_1000Hz, GODR_1000Hz);

  ares_ = icm_.getAres(AFS_2G);
  gres_ = icm_.getGres(GFS_250DPS);

  temp = 0.0;
  ax = ay = az = 0.0;
  gx = gy = gz = 0.0;
}

void ImuReceiver::update() {
  int16_t ICM42605Data[7];
  icm_.readData(ICM42605Data);

  temp = static_cast<float>(ICM42605Data[0]) * TEMP_RESOLUTION + TEMP_OFFSET;

  float ax_ =
      static_cast<float>(ICM42605Data[1]) * ares_ * GRAVITATIONAL_ACCELERATION;
  float ay_ =
      static_cast<float>(ICM42605Data[2]) * ares_ * GRAVITATIONAL_ACCELERATION;
  float az_ =
      -static_cast<float>(ICM42605Data[3]) * ares_ * GRAVITATIONAL_ACCELERATION;
  float gx_ = static_cast<float>(ICM42605Data[4]) * gres_ * DEGREE_TO_RADIAN;
  float gy_ = static_cast<float>(ICM42605Data[5]) * gres_ * DEGREE_TO_RADIAN;
  float gz_ = -static_cast<float>(ICM42605Data[6]) * gres_ * DEGREE_TO_RADIAN;

  switch (orientation_) {
    case Orientation::DEFAULT:
      ax = ax_;
      ay = ay_;
      az = az_;
      gx = gx_;
      gy = gy_;
      gz = gz_;
      break;
    case Orientation::X_LEFT_Z_DOWN:
      ax = ay_;
      ay = ax_;
      az = -az_;
      gx = gy_;
      gy = gx_;
      gz = -gz_;
      break;
    case Orientation::X_LEFT_Z_FORWARD:
      ax = az_;
      ay = ax_;
      az = ay_;
      gx = gz_;
      gy = gx_;
      gz = gy_;
      break;
  }
}