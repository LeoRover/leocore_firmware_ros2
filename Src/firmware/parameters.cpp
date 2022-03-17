#include "firmware/parameters.hpp"

void init_parameter_double(rclc_parameter_server_t* param_server,
                           const char* param_name, double default_value) {
  rclc_add_parameter(param_server, param_name, RCLC_PARAMETER_DOUBLE);
  rclc_parameter_set_double(param_server, param_name, default_value);
}

void init_parameter_int(rclc_parameter_server_t* param_server,
                        const char* param_name, int default_value) {
  rclc_add_parameter(param_server, param_name, RCLC_PARAMETER_INT);
  rclc_parameter_set_int(param_server, param_name, default_value);
}

void Parameters::init(rclc_parameter_server_t* param_server) {
  init_parameter_double(param_server, "wheels/encoder_resolution",
                        wheel_encoder_resolution);
  init_parameter_double(param_server, "wheels/torque_constant",
                        wheel_torque_constant);
  init_parameter_double(param_server, "wheels/pid/p", wheel_pid_p);
  init_parameter_double(param_server, "wheels/pid/i", wheel_pid_i);
  init_parameter_double(param_server, "wheels/pid/d", wheel_pid_d);
  init_parameter_double(param_server, "wheels/pwm_duty_limit",
                        wheel_pwm_duty_limit);
  init_parameter_double(param_server, "diff_drive/wheel_radius",
                        dd_wheel_radius);
  init_parameter_double(param_server, "diff_drive/wheel_separation",
                        dd_wheel_separation);
  init_parameter_double(param_server, "diff_drive/angular_velocity_multiplier",
                        dd_angular_velocity_multiplier);
  init_parameter_int(param_server, "diff_drive/input_timeout",
                     dd_input_timeout);
  init_parameter_double(param_server, "battery_min_voltage",
                        battery_min_voltage);
}