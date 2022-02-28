#include <sys/timespec.h>
#include <sys/types.h>

#include "stm32f4xx_hal.h"

int clock_gettime(clockid_t unused, struct timespec *tp) {
  (void)unused;

  uint32_t m = HAL_GetTick();

  tp->tv_sec = m / 1000;
  tp->tv_nsec = (m % 1000) * 1000000;

  return 0;
}