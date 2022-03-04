#include "firmware/imu_receiver.hpp"

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
  ax = static_cast<float>(ICM42605Data[2]) * ares_ * GRAVITATIONAL_ACCELERATION;
  ay = static_cast<float>(ICM42605Data[1]) * ares_ * GRAVITATIONAL_ACCELERATION;
  az = -static_cast<float>(ICM42605Data[3]) * ares_ * GRAVITATIONAL_ACCELERATION;
  gx = static_cast<float>(ICM42605Data[5]) * gres_ * DEGREE_TO_RADIAN;
  gy = static_cast<float>(ICM42605Data[4]) * gres_ * DEGREE_TO_RADIAN;
  gz = -static_cast<float>(ICM42605Data[6]) * gres_ * DEGREE_TO_RADIAN;
}