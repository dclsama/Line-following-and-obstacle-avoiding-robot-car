#include "encoder.h"
#include "board.h"

static int32_t enc_l = 0;
static int32_t enc_r = 0;
static int16_t last_l = 0;
static int16_t last_r = 0;

void encoder_init(void) {
  last_l = (int16_t)__HAL_TIM_GET_COUNTER(ENC_TIM_LEFT);
  last_r = (int16_t)__HAL_TIM_GET_COUNTER(ENC_TIM_RIGHT);
  enc_l = 0;
  enc_r = 0;
}

void encoder_update(void) {
  int16_t now_l = (int16_t)__HAL_TIM_GET_COUNTER(ENC_TIM_LEFT);
  int16_t now_r = (int16_t)__HAL_TIM_GET_COUNTER(ENC_TIM_RIGHT);
  enc_l += (int32_t)(now_l - last_l);
  enc_r += (int32_t)(now_r - last_r);
  last_l = now_l;
  last_r = now_r;
}

int32_t encoder_left(void) {
  return enc_l;
}

int32_t encoder_right(void) {
  return enc_r;
}
