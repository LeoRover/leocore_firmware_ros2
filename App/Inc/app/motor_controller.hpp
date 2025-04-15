#pragma once

#include <stdint.h>

#include "app/hal_compat.hpp"

#include "diff_drive_lib/motor_controller_interface.hpp"

struct MotorConfiguration {
  GPIO nsleep, phase, mode, fault;
  volatile uint32_t *enc_cnt;
  volatile uint32_t *pwm_ccr;
  volatile uint16_t *vpropi_adc;
  bool reverse_polarity;
};

class MotorController : public diff_drive_lib::MotorControllerInterface {
 public:
  MotorController(const MotorConfiguration &config) : config_(config){};

  void init() override;
  void setPWMDutyCycle(float pwm_duty) override;
  float getPWMDutyCycle() override;
  int32_t getEncoderCnt() override;
  void resetEncoderCnt() override;
  float getWindingCurrent() override;

 private:
  MotorConfiguration config_;

  float pwm_duty_ = 0.0F;
  uint8_t ticks_prev_quarter_ = 0;
  int32_t ticks_offset_ = 0;
};
