#include "firmware/parameters.hpp"

constexpr const char* wheel_encoder_resolution_param_name =
    "wheels/encoder_resolution";
constexpr const char* wheel_torque_constant_param_name =
    "wheels/torque_constant";
constexpr const char* wheel_pid_p_param_name = "wheels/pid/p";
constexpr const char* wheel_pid_i_param_name = "wheels/pid/i";
constexpr const char* wheel_pid_d_param_name = "wheels/pid/d";
constexpr const char* wheel_pwm_duty_limit_param_name = "wheels/pwm_duty_limit";
constexpr const char* mecanum_wheels_param_name = "mecanum_wheels";
constexpr const char* controller_wheel_radius_param_name =
    "controller/wheel_radius";
constexpr const char* controller_wheel_separation_param_name =
    "controller/wheel_separation";
constexpr const char* controller_wheel_base_param_name =
    "controller/wheel_base";
constexpr const char* controller_angular_velocity_multiplier_param_name =
    "controller/angular_velocity_multiplier";
constexpr const char* controller_input_timeout_param_name =
    "controller/input_timeout";
constexpr const char* battery_min_voltage_param_name = "battery_min_voltage";

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
                                    const char* param_name,
                                    int64_t default_value) {
  rcl_ret_t ret =
      rclc_add_parameter(param_server, param_name, RCLC_PARAMETER_INT);
  if (ret == RCL_RET_OK)
    return rclc_parameter_set_int(param_server, param_name, default_value);
  return ret;
}

inline rcl_ret_t init_parameter_bool(rclc_parameter_server_t* param_server,
                                     const char* param_name,
                                     bool default_value) {
  rcl_ret_t ret =
      rclc_add_parameter(param_server, param_name, RCLC_PARAMETER_BOOL);
  if (ret == RCL_RET_OK)
    return rclc_parameter_set_bool(param_server, param_name, default_value);
  return ret;
}

#define RCCHECK(fn) \
  if ((fn != RCL_RET_OK)) return false;

bool Parameters::init(rclc_parameter_server_t* param_server) {
  RCCHECK(init_parameter_double(param_server,
                                wheel_encoder_resolution_param_name,
                                wheel_encoder_resolution))
  RCCHECK(init_parameter_double(param_server, wheel_torque_constant_param_name,
                                wheel_torque_constant))
  RCCHECK(
      init_parameter_double(param_server, wheel_pid_p_param_name, wheel_pid_p))
  RCCHECK(
      init_parameter_double(param_server, wheel_pid_i_param_name, wheel_pid_i))
  RCCHECK(
      init_parameter_double(param_server, wheel_pid_d_param_name, wheel_pid_d))
  RCCHECK(init_parameter_double(param_server, wheel_pwm_duty_limit_param_name,
                                wheel_pwm_duty_limit))
  RCCHECK(init_parameter_bool(param_server, mecanum_wheels_param_name,
                              mecanum_wheels))
  RCCHECK(init_parameter_double(
      param_server, controller_wheel_radius_param_name, robot_wheel_radius))
  RCCHECK(init_parameter_double(param_server,
                                controller_wheel_separation_param_name,
                                robot_wheel_separation))
  RCCHECK(init_parameter_double(param_server, controller_wheel_base_param_name,
                                robot_wheel_base))
  RCCHECK(init_parameter_double(
      param_server, controller_angular_velocity_multiplier_param_name,
      robot_angular_velocity_multiplier))
  RCCHECK(init_parameter_int(param_server, controller_input_timeout_param_name,
                             robot_input_timeout))
  RCCHECK(init_parameter_double(param_server, battery_min_voltage_param_name,
                                battery_min_voltage))
  return true;
}

inline void get_parameter_double(rclc_parameter_server_t* param_server,
                                 const char* param_name, float* output) {
  double tmp;
  rclc_parameter_get_double(param_server, param_name, &tmp);
  *output = static_cast<float>(tmp);
}

inline void get_parameter_int(rclc_parameter_server_t* param_server,
                              const char* param_name, int* output) {
  int64_t tmp;
  rclc_parameter_get_int(param_server, param_name, &tmp);
  *output = static_cast<int>(tmp);
}

void Parameters::update(rclc_parameter_server_t* param_server) {
  get_parameter_double(param_server, wheel_encoder_resolution_param_name,
                       &wheel_encoder_resolution);
  get_parameter_double(param_server, wheel_torque_constant_param_name,
                       &wheel_torque_constant);
  get_parameter_double(param_server, wheel_pid_p_param_name, &wheel_pid_p);
  get_parameter_double(param_server, wheel_pid_i_param_name, &wheel_pid_i);
  get_parameter_double(param_server, wheel_pid_d_param_name, &wheel_pid_d);
  get_parameter_double(param_server, wheel_pwm_duty_limit_param_name,
                       &wheel_pwm_duty_limit);
  rclc_parameter_get_bool(param_server, mecanum_wheels_param_name,
                          &mecanum_wheels);
  get_parameter_double(param_server, controller_wheel_radius_param_name,
                       &robot_wheel_radius);
  get_parameter_double(param_server, controller_wheel_separation_param_name,
                       &robot_wheel_separation);
  get_parameter_double(param_server, controller_wheel_base_param_name,
                       &robot_wheel_base);
  get_parameter_double(param_server,
                       controller_angular_velocity_multiplier_param_name,
                       &robot_angular_velocity_multiplier);
  get_parameter_int(param_server, controller_input_timeout_param_name,
                    &robot_input_timeout);
  get_parameter_double(param_server, battery_min_voltage_param_name,
                       &battery_min_voltage);
}