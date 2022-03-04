#pragma once

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

#include <ICM42605.hpp>

class ImuReceiver {
 public:
  explicit ImuReceiver(I2C_HandleTypeDef* i2c) : icm_(i2c) {}

  void init();
  void update();

  float temp;        // temperature
  float ax, ay, az;  // accelerometer data
  float gx, gy, gz;  // gyroscope data

 private:
  ICM42605 icm_;
  float ares_, gres_;
};