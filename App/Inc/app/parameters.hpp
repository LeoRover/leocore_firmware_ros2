#pragma once

#include "rclc_parameter/rclc_parameter.h"

#include "diff_drive_lib/robot_controller.hpp"

struct Parameters : diff_drive_lib::RobotParams {
  // Override inherited parameters
  Parameters() {
    // Wheel
    wheel_encoder_resolution = 878.4F;
    wheel_torque_constant = 1.17647F;
    wheel_pid_p = 0.0F;
    wheel_pid_i = 0.005F;
    wheel_pid_d = 0.0F;
    wheel_pwm_duty_limit = 100.0F;

    robot_wheel_radius = 0.0625F;
    robot_wheel_separation = 0.358F;
    robot_wheel_base = 0.3052F;
    robot_angular_velocity_multiplier = 1.76F;
    robot_input_timeout = 500;
  }

  float battery_min_voltage = 10.0;

  bool mecanum_wheels = false;

  int leo_hardware_version = 1;  // Leo Rover v1.8

  bool init(rclc_parameter_server_t* param_server);
  void update(rclc_parameter_server_t* param_server);
};
