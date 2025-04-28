#include <cstdint>
#include <cstdlib>

// Defined in app.cpp
extern volatile uint16_t adc_buff[6];

static int64_t rand_next;

extern "C" void srand(unsigned int seed) {
  rand_next = adc_buff[0] ^ adc_buff[1] ^ adc_buff[2] ^ adc_buff[3] ^
              adc_buff[4] ^ adc_buff[5] ^ seed;
}

extern "C" int rand() {
  // Newlib's rand() implementation
  rand_next = rand_next * 6364136223846793005LL + 1;
  return (int)((rand_next >> 32) & RAND_MAX);
}