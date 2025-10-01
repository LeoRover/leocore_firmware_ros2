#pragma once

#include "i2c.h"
#include "main.h"
#include "usart.h"

#include "diff_drive_lib/robot_controller.hpp"

#include "app/hal_compat.hpp"
#include "app/motor_controller.hpp"

extern volatile uint16_t adc_buff[6];  // TODO: Move somewhere else

// UART used for micro-ROS communication
static constexpr UART_HandleTypeDef& UROS_UART = huart1;

// Size of the UART buffers used for micro-ROS communication
constexpr size_t UROS_RBUFFER_SIZE = 2048;
constexpr size_t UROS_TBUFFER_SIZE = 2048;

// Domain ID used for ROS communication
// When set to 255 it is automatically overridden by the uROS agent
constexpr size_t ROS_DOMAIN_ID = 255;

// Name of the ROS node
constexpr const char* ROS_NODE_NAME = "firmware";

// Namespace of the ROS node
constexpr const char* ROS_NAMESPACE = "";

// I2C used for IMU communication
static constexpr I2C_HandleTypeDef& IMU_I2C = hi2c1;

// The timer CCR value corresponding to 100% PWM duty cycle
constexpr uint16_t PWM_RANGE = 1000;

// Number of encoder readings to remember when estimating the wheel velocity
// constexpr uint32_t VELOCITY_ROLLING_WINDOW_SIZE = 10;

// The period (in number of calls to the update() function) at which the battery
// voltage is probed
constexpr uint8_t BATTERY_PROBE_PERIOD = 10;

// Number of battery voltage readings to average
constexpr uint32_t BATTERY_BUFFER_SIZE = 300;

// Informative LED GPIO
constexpr GPIO LED = {LED_GPIO_Port, LED_Pin};

// The period (in milliseconds) between calls to the update() function
constexpr uint16_t UPDATE_PERIOD = 10;

// The periods (in number of calls to the update() function) at which different
// data is publihed on the ROS topics
constexpr uint8_t BATTERY_PUB_PERIOD = 10;
// constexpr uint8_t JOINTS_PUB_PERIOD = 5;
// constexpr uint8_t ODOM_PUB_PERIOD = 5;
constexpr uint8_t IMU_PUB_PERIOD = 1;
constexpr uint8_t PARAM_TRIGGER_PUB_PERIOD = 100;

// The time after which the firmware will boot with default parameter values
constexpr uint32_t BOOT_TIMEOUT = 20000;

// Raw value of the Battery ADC
static volatile uint16_t& BATTERY_ADC = adc_buff[4];

// How much Volts per precision of Battery ADC
// 0-32 V range, 12 bit precision
constexpr float BATTERY_ADC_TO_VOLTAGE = 32.0F / 4095.0F;

// How much Amperes per precision of VPROPI ADC
// 0-2.5 A range, 12 bit precision
constexpr float VPROPI_ADC_TO_CURRENT = 2.5F / 4095.0F;
