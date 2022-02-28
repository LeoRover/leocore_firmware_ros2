#include <rmw_microros/custom_transport.h>
#include <uxr/client/transport.h>

#include "microros/uart_transport.h"

#define UART_DMA_BUFFER_SIZE 2048

static uint8_t dma_buffer[UART_DMA_BUFFER_SIZE];
static size_t dma_head = 0, dma_tail = 0;

bool uart_transport_open(struct uxrCustomTransport* transport) {
  UART_HandleTypeDef* uart = (UART_HandleTypeDef*)transport->args;
  HAL_UART_Receive_DMA(uart, dma_buffer, UART_DMA_BUFFER_SIZE);
  return true;
}

bool uart_transport_close(struct uxrCustomTransport* transport) {
  UART_HandleTypeDef* uart = (UART_HandleTypeDef*)transport->args;
  HAL_UART_DMAStop(uart);
  return true;
}

size_t uart_transport_write(struct uxrCustomTransport* transport,
                            const uint8_t* buf, size_t len, uint8_t* err) {
  UART_HandleTypeDef* uart = (UART_HandleTypeDef*)transport->args;

  HAL_StatusTypeDef ret;
  if (uart->gState == HAL_UART_STATE_READY) {
    ret = HAL_UART_Transmit_DMA(uart, buf, len);
    while (ret == HAL_OK && uart->gState != HAL_UART_STATE_READY) {
      HAL_Delay(1);
    }

    return (ret == HAL_OK) ? len : 0;
  } else {
    return 0;
  }
}

size_t uart_transport_read(struct uxrCustomTransport* transport, uint8_t* buf,
                           size_t len, int timeout, uint8_t* err) {
  UART_HandleTypeDef* uart = (UART_HandleTypeDef*)transport->args;

  int ms_used = 0;
  do {
    __disable_irq();
    dma_tail = UART_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(uart->hdmarx);
    __enable_irq();
    ms_used++;
    HAL_Delay(1);
  } while (dma_head == dma_tail && ms_used < timeout);

  size_t wrote = 0;
  while ((dma_head != dma_tail) && (wrote < len)) {
    buf[wrote] = dma_buffer[dma_head];
    dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
    wrote++;
  }

  return wrote;
}

void rmw_uros_set_uart_transport(UART_HandleTypeDef* huart) {
  rmw_uros_set_custom_transport(true, (void*)huart, uart_transport_open,
                                uart_transport_close, uart_transport_write,
                                uart_transport_read);
}