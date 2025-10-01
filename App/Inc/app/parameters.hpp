#pragma once

#include "rclc_parameter/rclc_parameter.h"

#include "diff_drive_lib/robot_controller.hpp"

struct Parameters : diff_drive_lib::RobotParams {
  // Override inherited parameters
  Parameters() {}

  float battery_min_voltage = 10.0;

  int leo_hardware_version = 1;  // Leo Rover v1.8

  bool init(rclc_parameter_server_t* param_server);
  void update(rclc_parameter_server_t* param_server);
};
