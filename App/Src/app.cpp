#include "adc.h"
#include "tim.h"

volatile uint16_t adc_buff[5];

void setup() {}

void loop() {}

void update() {}

extern "C" void app_main() {
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  HAL_TIM_Base_Start_IT(&htim11);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buff, 5);

  setup();

  while (1) {
    loop();
  }
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM11) {
    update();
  }
}
