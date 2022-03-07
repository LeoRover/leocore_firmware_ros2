#include <rmw_microros/custom_transport.h>
#include <uxr/client/transport.h>

#include "microros/uart_transport.h"

#define DMA_RBUFFER_SIZE 2048
#define DMA_TBUFFER_SIZE 2048

static uint8_t dma_rbuffer[DMA_RBUFFER_SIZE];
static uint16_t dma_rhead = 0, dma_rtail = 0;

static uint8_t dma_tbuffer[DMA_TBUFFER_SIZE];
static volatile uint16_t dma_thead = 0, dma_thead_next = 0, dma_ttail = 0;

bool uart_transport_open(struct uxrCustomTransport* transport) {
  UART_HandleTypeDef* uart = (UART_HandleTypeDef*)transport->args;
  HAL_UART_Receive_DMA(uart, dma_rbuffer, DMA_RBUFFER_SIZE);
  return true;
}

bool uart_transport_close(struct uxrCustomTransport* transport) {
  UART_HandleTypeDef* uart = (UART_HandleTypeDef*)transport->args;
  HAL_UART_DMAStop(uart);
  return true;
}

static void uart_flush(UART_HandleTypeDef* uart) {
  static volatile bool mutex = false;

  if ((uart->gState == HAL_UART_STATE_READY) && !mutex) {
    mutex = true;

    if (dma_thead != dma_ttail) {
      uint16_t len = dma_thead < dma_ttail ? dma_ttail - dma_thead
                                           : DMA_TBUFFER_SIZE - dma_thead;
      dma_thead_next = (dma_thead + len) & (DMA_TBUFFER_SIZE - 1);
      HAL_UART_Transmit_DMA(uart, dma_tbuffer + dma_thead, len);
    }
    mutex = false;
  }
}

static inline uint16_t get_buffer_available() {
  return dma_thead <= dma_ttail ? DMA_TBUFFER_SIZE - dma_ttail + dma_thead
                                : dma_thead - dma_ttail;
}

size_t uart_transport_write(struct uxrCustomTransport* transport,
                            const uint8_t* buf, size_t len, uint8_t* err) {
  UART_HandleTypeDef* uart = (UART_HandleTypeDef*)transport->args;

  uint16_t n = len;
  uint16_t buffer_available = dma_thead <= dma_ttail
                                  ? DMA_TBUFFER_SIZE - dma_ttail + dma_thead
                                  : dma_thead - dma_ttail;
  if (n > buffer_available) n = buffer_available;

  uint16_t n_tail =
      n <= DMA_TBUFFER_SIZE - dma_ttail ? n : DMA_TBUFFER_SIZE - dma_ttail;

  memcpy(dma_tbuffer + dma_ttail, buf, n_tail);

  if (n != n_tail) {
    memcpy(dma_tbuffer, buf + n_tail, n - n_tail);
  }

  dma_ttail = (dma_ttail + n) & (DMA_TBUFFER_SIZE - 1);

  uart_flush(uart);

  return n;
}

void uart_transfer_complete_callback(UART_HandleTypeDef* uart) {
  dma_thead = dma_thead_next;
  uart_flush(uart);
}

size_t uart_transport_read(struct uxrCustomTransport* transport, uint8_t* buf,
                           size_t len, int timeout, uint8_t* err) {
  UART_HandleTypeDef* uart = (UART_HandleTypeDef*)transport->args;

  int ms_used = 0;
  do {
    __disable_irq();
    dma_rtail = DMA_RBUFFER_SIZE - __HAL_DMA_GET_COUNTER(uart->hdmarx);
    __enable_irq();

    uint16_t data_available = dma_rhead <= dma_rtail
                                  ? dma_rtail - dma_rhead
                                  : DMA_RBUFFER_SIZE - dma_rhead + dma_rtail;

    if (data_available >= len) break;

    HAL_Delay(1);
    ms_used++;
  } while (ms_used < timeout);

  size_t wrote = 0;
  while ((dma_rhead != dma_rtail) && (wrote < len)) {
    buf[wrote++] = dma_rbuffer[dma_rhead];
    dma_rhead = (dma_rhead + 1) & (DMA_RBUFFER_SIZE - 1);
  }

  return wrote;
}

void rmw_uros_set_uart_transport(UART_HandleTypeDef* huart) {
  rmw_uros_set_custom_transport(true, (void*)huart, uart_transport_open,
                                uart_transport_close, uart_transport_write,
                                uart_transport_read);
}