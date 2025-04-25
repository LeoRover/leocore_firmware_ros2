#pragma once

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

#include <ICM42605.hpp>

class ImuReceiver {
 public:
  enum class Orientation {
    // Default orientation (X forward, Y left, Z up)
    DEFAULT = 0,

    // Leo Rover v1.8 orientation (X left, Y forward, Z down)
    X_LEFT_Z_DOWN = 1,

    // Leo Rover v1.9 orientation (X left, Y up, Z forward)
    X_LEFT_Z_FORWARD = 2,
  };

  explicit ImuReceiver(I2C_HandleTypeDef* i2c) : icm_(i2c), orientation_(Orientation::DEFAULT) {}

  void init();
  void update();

  void setOrientation(Orientation orientation) { orientation_ = orientation; }

  float temp;        // temperature
  float ax, ay, az;  // accelerometer data
  float gx, gy, gz;  // gyroscope data

 private:
  ICM42605 icm_;
  float ares_, gres_;
  Orientation orientation_;
};