#include "firmware/parameters.hpp"

inline rcl_ret_t init_parameter_double(rclc_parameter_server_t* param_server,
                                       const char* param_name,
                                       double default_value) {
  rcl_ret_t ret =
      rclc_add_parameter(param_server, param_name, RCLC_PARAMETER_DOUBLE);
  if (ret == RCL_RET_OK)
    return rclc_parameter_set_double(param_server, param_name, default_value);
  return ret;
}

inline rcl_ret_t init_parameter_int(rclc_parameter_server_t* param_server,
                                    const char* param_name, int default_value) {
  rcl_ret_t ret =
      rclc_add_parameter(param_server, param_name, RCLC_PARAMETER_INT);
  if (ret == RCL_RET_OK)
    return rclc_parameter_set_int(param_server, param_name, default_value);
  return ret;
}

#define RCCHECK(fn) \
  if ((fn != RCL_RET_OK)) return false;

bool Parameters::init(rclc_parameter_server_t* param_server) {
  RCCHECK(init_parameter_double(param_server, "wheels/encoder_resolution",
                                wheel_encoder_resolution))
  RCCHECK(init_parameter_double(param_server, "wheels/torque_constant",
                        wheel_torque_constant))
  RCCHECK(init_parameter_double(param_server, "wheels/pid/p", wheel_pid_p))
  RCCHECK(init_parameter_double(param_server, "wheels/pid/i", wheel_pid_i))
  RCCHECK(init_parameter_double(param_server, "wheels/pid/d", wheel_pid_d))
  RCCHECK(init_parameter_double(param_server, "wheels/pwm_duty_limit",
                        wheel_pwm_duty_limit))
  RCCHECK(init_parameter_double(param_server, "diff_drive/wheel_radius",
                        dd_wheel_radius))
  RCCHECK(init_parameter_double(param_server, "diff_drive/wheel_separation",
                        dd_wheel_separation))
  RCCHECK(init_parameter_double(param_server, "diff_drive/angular_velocity_multiplier",
                        dd_angular_velocity_multiplier))
  RCCHECK(init_parameter_int(param_server, "diff_drive/input_timeout",
                     dd_input_timeout))
  RCCHECK(init_parameter_double(param_server, "battery_min_voltage",
                        battery_min_voltage))
  return true;
}