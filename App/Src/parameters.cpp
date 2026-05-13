#include "app/parameters.hpp"

#include <cmath>
#include <cstdint>

constexpr const char* wheel_encoder_resolution_param_name =
    "wheels.encoder_resolution";
constexpr const char* wheel_torque_constant_param_name =
    "wheels.torque_constant";
constexpr const char* wheel_pid_p_param_name = "wheels.pid.kp";
constexpr const char* wheel_pid_i_param_name = "wheels.pid.ki";
constexpr const char* wheel_pid_d_param_name = "wheels.pid.kd";
constexpr const char* wheel_max_voltage_param_name = "wheels.max_voltage";
constexpr const char* mecanum_wheels_param_name = "mecanum_wheels";
constexpr const char* controller_wheel_radius_param_name =
    "controller.wheel_radius";
constexpr const char* controller_wheel_separation_param_name =
    "controller.wheel_separation";
constexpr const char* controller_wheel_base_param_name =
    "controller.wheel_base";
constexpr const char* controller_angular_velocity_multiplier_param_name =
    "controller.angular_velocity_multiplier";
constexpr const char* controller_input_timeout_param_name =
    "controller.input_timeout";
constexpr const char* controller_linear_acceleration_param_name =
    "controller.linear_acceleration";
constexpr const char* controller_linear_deceleration_param_name =
    "controller.linear_deceleration";
constexpr const char* controller_angular_acceleration_param_name =
    "controller.angular_acceleration";
constexpr const char* controller_angular_deceleration_param_name =
    "controller.angular_deceleration";
constexpr const char* battery_min_voltage_param_name = "battery_min_voltage";
constexpr const char* leo_hardware_version_param_name = "leo_hardware_version";

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
  RCCHECK(init_parameter_double(param_server, wheel_max_voltage_param_name,
                                wheel_max_voltage))
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
  RCCHECK(init_parameter_double(param_server,
                                controller_linear_acceleration_param_name,
                                robot_linear_acceleration))
  RCCHECK(init_parameter_double(param_server,
                                controller_linear_deceleration_param_name,
                                robot_linear_deceleration))
  RCCHECK(init_parameter_double(param_server,
                                controller_angular_acceleration_param_name,
                                robot_angular_acceleration))
  RCCHECK(init_parameter_double(param_server,
                                controller_angular_deceleration_param_name,
                                robot_angular_deceleration))
  RCCHECK(init_parameter_double(param_server, battery_min_voltage_param_name,
                                battery_min_voltage))
  RCCHECK(init_parameter_int(param_server, leo_hardware_version_param_name,
                             leo_hardware_version))
  return true;
}

inline void get_parameter_double(rclc_parameter_server_t* param_server,
                                 const char* param_name, float* output,
                                 float min_value = -1e9F) {
  double tmp;
  rclc_parameter_get_double(param_server, param_name, &tmp);
  float val = static_cast<float>(tmp);
  if (std::isfinite(val)) {
    *output = val > min_value ? val : min_value;
  }
}

inline void get_parameter_int(rclc_parameter_server_t* param_server,
                              const char* param_name, int* output,
                              int min_value = -1000000) {
  int64_t tmp;
  rclc_parameter_get_int(param_server, param_name, &tmp);
  if (tmp > INT32_MAX) {
    tmp = INT32_MAX;
  } else if (tmp < INT32_MIN) {
    tmp = INT32_MIN;
  }
  int val = static_cast<int>(tmp);
  *output = val > min_value ? val : min_value;
}

constexpr float MIN_POSITIVE = 1e-3F;

void Parameters::update(rclc_parameter_server_t* param_server) {
  get_parameter_double(param_server, wheel_encoder_resolution_param_name,
                       &wheel_encoder_resolution, MIN_POSITIVE);
  get_parameter_double(param_server, wheel_torque_constant_param_name,
                       &wheel_torque_constant);
  get_parameter_double(param_server, wheel_pid_p_param_name, &wheel_pid_p);
  get_parameter_double(param_server, wheel_pid_i_param_name, &wheel_pid_i);
  get_parameter_double(param_server, wheel_pid_d_param_name, &wheel_pid_d);
  get_parameter_double(param_server, wheel_max_voltage_param_name,
                       &wheel_max_voltage, 0.0F);
  rclc_parameter_get_bool(param_server, mecanum_wheels_param_name,
                          &mecanum_wheels);
  get_parameter_double(param_server, controller_wheel_radius_param_name,
                       &robot_wheel_radius, MIN_POSITIVE);
  get_parameter_double(param_server, controller_wheel_separation_param_name,
                       &robot_wheel_separation, MIN_POSITIVE);
  get_parameter_double(param_server, controller_wheel_base_param_name,
                       &robot_wheel_base, 0.0F);
  get_parameter_double(param_server,
                       controller_angular_velocity_multiplier_param_name,
                       &robot_angular_velocity_multiplier, MIN_POSITIVE);
  get_parameter_int(param_server, controller_input_timeout_param_name,
                    &robot_input_timeout, 0);
  get_parameter_double(param_server, controller_linear_acceleration_param_name,
                       &robot_linear_acceleration, 0.0F);
  get_parameter_double(param_server, controller_linear_deceleration_param_name,
                       &robot_linear_deceleration, 0.0F);
  get_parameter_double(param_server, controller_angular_acceleration_param_name,
                       &robot_angular_acceleration, 0.0F);
  get_parameter_double(param_server, controller_angular_deceleration_param_name,
                       &robot_angular_deceleration, 0.0F);
  get_parameter_double(param_server, battery_min_voltage_param_name,
                       &battery_min_voltage, 0.0F);
  get_parameter_int(param_server, leo_hardware_version_param_name,
                    &leo_hardware_version, 1);
}