#pragma once

#include "i2c.h"
#include "main.h"
#include "mainf.h"
#include "usart.h"

#include "diff_drive_controller.hpp"

#include "firmware/hal_compat.hpp"
#include "firmware/motor_controller.hpp"

// UART used for micro-ROS communication
static UART_HandleTypeDef& UROS_UART = huart1;

// I2C used for IMU communication
static I2C_HandleTypeDef& IMU_I2C = hi2c1;

// The timer CCR value corresponding to 100% PWM duty cycle
const uint16_t PWM_RANGE = 1000;

// Number of encoder readings to remember when estimating the wheel velocity
const uint32_t ENCODER_BUFFER_SIZE = 10;

// The period (in number of calls to the update() function) at which the battery
// voltage is probed
const uint8_t BATTERY_PROBE_PERIOD = 10;

// Number of battery voltage readings to average
const uint32_t BATTERY_BUFFER_SIZE = 300;

// Informative LED GPIO
const GPIO LED = {LED_GPIO_Port, LED_Pin};

// The period (in milliseconds) between calls to the update() function
const uint16_t UPDATE_PERIOD = 10;

// The periods (in number of calls to the update() function) at which different
// data is publihed on the ROS topics
const uint8_t BATTERY_PUB_PERIOD = 10;
const uint8_t JOINTS_PUB_PERIOD = 5;
const uint8_t ODOM_PUB_PERIOD = 5;
const uint8_t IMU_PUB_PERIOD = 1;

// Raw value of the Battery ADC
static volatile uint16_t& BATTERY_ADC = adc_buff[4];

// How much Volts per precision of Battery ADC
// 0-32 V range, 12 bit precision
const float BATTERY_ADC_TO_VOLTAGE = 32.0F / 4095.0F;

// How much Amperes per precision of VPROPI ADC
// 0-2.5 A range, 12 bit precision
const float VPROPI_ADC_TO_CURRENT = 2.5F / 4095.0F;

// Motor driver configurations
const MotorConfiguration MOT_A_CONFIG = {
    .nsleep = {H4_NSLEEP_GPIO_Port, H4_NSLEEP_Pin},
    .phase = {H4_PHASE_GPIO_Port, H4_PHASE_Pin},
    .mode = {H4_MODE_GPIO_Port, H4_MODE_Pin},
    .fault = {H4_FAULT_GPIO_Port, H4_FAULT_Pin},
    .enc_cnt = &TIM5->CNT,
    .pwm_ccr = &TIM1->CCR4,
    .vpropi_adc = &adc_buff[3],
    .reverse_polarity = false,
};

const MotorConfiguration MOT_B_CONFIG = {
    .nsleep = {H3_NSLEEP_GPIO_Port, H3_NSLEEP_Pin},
    .phase = {H3_PHASE_GPIO_Port, H3_PHASE_Pin},
    .mode = {H3_MODE_GPIO_Port, H3_MODE_Pin},
    .fault = {H3_FAULT_GPIO_Port, H3_FAULT_Pin},
    .enc_cnt = &TIM4->CNT,
    .pwm_ccr = &TIM9->CCR2,
    .vpropi_adc = &adc_buff[2],
    .reverse_polarity = false,
};

const MotorConfiguration MOT_C_CONFIG = {
    .nsleep = {H1_NSLEEP_GPIO_Port, H1_NSLEEP_Pin},
    .phase = {H1_PHASE_GPIO_Port, H1_PHASE_Pin},
    .mode = {H1_MODE_GPIO_Port, H1_MODE_Pin},
    .fault = {H1_FAULT_GPIO_Port, H1_FAULT_Pin},
    .enc_cnt = &TIM3->CNT,
    .pwm_ccr = &TIM1->CCR1,
    .vpropi_adc = &adc_buff[0],
    .reverse_polarity = true,
};

const MotorConfiguration MOT_D_CONFIG = {
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

const DiffDriveConfiguration DD_CONFIG = {
    .wheel_FL_conf =
        {
            .motor = MotC,
            .velocity_rolling_window_size = 10,
        },
    .wheel_RL_conf =
        {
            .motor = MotD,
            .velocity_rolling_window_size = 10,
        },
    .wheel_FR_conf =
        {
            .motor = MotA,
            .velocity_rolling_window_size = 10,
        },
    .wheel_RR_conf =
        {
            .motor = MotB,
            .velocity_rolling_window_size = 10,
        },
};