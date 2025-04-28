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
constexpr uint32_t VELOCITY_ROLLING_WINDOW_SIZE = 10;

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
constexpr uint8_t JOINTS_PUB_PERIOD = 5;
constexpr uint8_t ODOM_PUB_PERIOD = 5;
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

// Motor driver configurations
constexpr MotorConfiguration MOT_A_CONFIG = {
    .nsleep = {H4_NSLEEP_GPIO_Port, H4_NSLEEP_Pin},
    .phase = {H4_PHASE_GPIO_Port, H4_PHASE_Pin},
    .mode = {H4_MODE_GPIO_Port, H4_MODE_Pin},
    .fault = {H4_FAULT_GPIO_Port, H4_FAULT_Pin},
    .enc_cnt = &TIM5->CNT,
    .pwm_ccr = &TIM1->CCR4,
    .vpropi_adc = &adc_buff[3],
    .reverse_polarity = false,
};

constexpr MotorConfiguration MOT_B_CONFIG = {
    .nsleep = {H3_NSLEEP_GPIO_Port, H3_NSLEEP_Pin},
    .phase = {H3_PHASE_GPIO_Port, H3_PHASE_Pin},
    .mode = {H3_MODE_GPIO_Port, H3_MODE_Pin},
    .fault = {H3_FAULT_GPIO_Port, H3_FAULT_Pin},
    .enc_cnt = &TIM4->CNT,
    .pwm_ccr = &TIM9->CCR2,
    .vpropi_adc = &adc_buff[2],
    .reverse_polarity = false,
};

constexpr MotorConfiguration MOT_C_CONFIG = {
    .nsleep = {H1_NSLEEP_GPIO_Port, H1_NSLEEP_Pin},
    .phase = {H1_PHASE_GPIO_Port, H1_PHASE_Pin},
    .mode = {H1_MODE_GPIO_Port, H1_MODE_Pin},
    .fault = {H1_FAULT_GPIO_Port, H1_FAULT_Pin},
    .enc_cnt = &TIM3->CNT,
    .pwm_ccr = &TIM1->CCR1,
    .vpropi_adc = &adc_buff[0],
    .reverse_polarity = true,
};

constexpr MotorConfiguration MOT_D_CONFIG = {
    .nsleep = {H2_NSLEEP_GPIO_Port, H2_NSLEEP_Pin},
    .phase = {H2_PHASE_GPIO_Port, H2_PHASE_Pin},
    .mode = {H2_MODE_GPIO_Port, H2_MODE_Pin},
    .fault = {H2_FAULT_GPIO_Port, H2_FAULT_Pin},
    .enc_cnt = &TIM2->CNT,
    .pwm_ccr = &TIM9->CCR1,
    .vpropi_adc = &adc_buff[1],
    .reverse_polarity = true,
};

extern MotorController MotA;
extern MotorController MotB;
extern MotorController MotC;
extern MotorController MotD;

// Robot configuration for Leo Rover v1.8 or earlier
constexpr diff_drive_lib::RobotConfiguration ROBOT_CONFIG_108 = {
    .wheel_FL_conf =
        {
            .motor = MotC,
            .op_mode = diff_drive_lib::WheelOperationMode::VELOCITY,
        },
    .wheel_RL_conf =
        {
            .motor = MotD,
            .op_mode = diff_drive_lib::WheelOperationMode::VELOCITY,
        },
    .wheel_FR_conf =
        {
            .motor = MotA,
            .op_mode = diff_drive_lib::WheelOperationMode::VELOCITY,
        },
    .wheel_RR_conf =
        {
            .motor = MotB,
            .op_mode = diff_drive_lib::WheelOperationMode::VELOCITY,
        },
};

// Robot configuration for Leo Rover v1.9 or later
constexpr diff_drive_lib::RobotConfiguration ROBOT_CONFIG = {
    .wheel_FL_conf =
        {
            .motor = MotD,
            .op_mode = diff_drive_lib::WheelOperationMode::VELOCITY,
        },
    .wheel_RL_conf =
        {
            .motor = MotC,
            .op_mode = diff_drive_lib::WheelOperationMode::VELOCITY,
        },
    .wheel_FR_conf =
        {
            .motor = MotB,
            .op_mode = diff_drive_lib::WheelOperationMode::VELOCITY,
        },
    .wheel_RR_conf =
        {
            .motor = MotA,
            .op_mode = diff_drive_lib::WheelOperationMode::VELOCITY,
        },
};
