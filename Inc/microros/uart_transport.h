#pragma once

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void rmw_uros_set_uart_transport(UART_HandleTypeDef*);
void uart_transfer_complete_callback(UART_HandleTypeDef*);

#ifdef __cplusplus
}
#endif