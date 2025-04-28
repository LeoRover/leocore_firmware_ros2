#include <cstdint>
#include <cstdlib>

// Defined in app.cpp
extern volatile uint16_t adc_buff[6];

static int64_t rand_next;

extern "C" void srand(unsigned int seed) {
  rand_next = seed;
  for (int i = 0; i < 6; ++i) {
    rand_next ^= ((int64_t)adc_buff[i]) << (i * 10); // Spread across 64 bits
  }
}

extern "C" int rand() {
  // Newlib's rand() implementation
  rand_next = rand_next * 6364136223846793005LL + 1;
  return (int)((rand_next >> 32) & RAND_MAX);
}