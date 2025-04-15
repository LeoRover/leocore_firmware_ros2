#pragma once

#include <cstdint>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"

struct GPIO {
  GPIO_TypeDef* port;
  uint16_t pin;
};

inline void gpio_set(const GPIO& gpio) {
  HAL_GPIO_WritePin(gpio.port, gpio.pin, GPIO_PIN_SET);
}

inline void gpio_reset(const GPIO& gpio) {
  HAL_GPIO_WritePin(gpio.port, gpio.pin, GPIO_PIN_RESET);
}

inline void gpio_toggle(const GPIO& gpio) {
  HAL_GPIO_TogglePin(gpio.port, gpio.pin);
}

inline uint32_t time() { return HAL_GetTick(); }

inline void reset() { NVIC_SystemReset(); }

inline void delay(uint32_t delay_ms) { HAL_Delay(delay_ms); }