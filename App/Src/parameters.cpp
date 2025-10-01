#include "app/parameters.hpp"

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
  RCCHECK(init_parameter_double(param_server, battery_min_voltage_param_name,
                                battery_min_voltage))
  RCCHECK(init_parameter_int(param_server, leo_hardware_version_param_name,
                             leo_hardware_version))
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
  get_parameter_double(param_server, battery_min_voltage_param_name,
                       &battery_min_voltage);
  get_parameter_int(param_server, leo_hardware_version_param_name,
                    &leo_hardware_version);
}