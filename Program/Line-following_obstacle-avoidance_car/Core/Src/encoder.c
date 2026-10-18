#include "encoder.h"
#include "board.h"

static int32_t  enc_l = 0;
static int32_t  enc_r = 0;
static uint16_t last_raw_l = 0;   /* 上一次的原始计数值(16位) */
static uint16_t last_raw_r = 0;

void encoder_init(void) {
  last_raw_l = (uint16_t)__HAL_TIM_GET_COUNTER(ENC_TIM_LEFT);
  last_raw_r = (uint16_t)__HAL_TIM_GET_COUNTER(ENC_TIM_RIGHT);
  enc_l = 0;
  enc_r = 0;
}

void encoder_update(void) {
  uint16_t now_l = (uint16_t)__HAL_TIM_GET_COUNTER(ENC_TIM_LEFT);
  uint16_t now_r = (uint16_t)__HAL_TIM_GET_COUNTER(ENC_TIM_RIGHT);

  /* 16位计数器回绕的正确算法:
   * 不能写 (int16_t)now - (int16_t)last —— 两个 int16_t 会先提升成 int,
   * 做的是 32 位减法. 计数器从 32767 跨到 -32768 时会算出 -65535
   * (正确值应是 +1), 约每 65536 个计数(几米)出现一次, 会让 turn_diff()
   * 瞬间爆表, 避障子阶段提前结束, 里程判据也会失真.
   *
   * 正确做法: 先按 uint16_t 相减(自动模 65536 回绕), 再解释成有符号增量. */
  enc_l += (int16_t)(now_l - last_raw_l);
  enc_r += (int16_t)(now_r - last_raw_r);

  last_raw_l = now_l;
  last_raw_r = now_r;
}

int32_t encoder_left(void) {
  return enc_l;
}

int32_t encoder_right(void) {
  return enc_r;
}
