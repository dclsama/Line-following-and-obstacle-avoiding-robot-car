#include "ultrasonic.h"
#include "board.h"
#include "utils.h"

#define US_NUM 5

typedef struct {
  GPIO_TypeDef *trig_port;
  uint16_t      trig_pin;
  GPIO_TypeDef *echo_port;
  uint16_t      echo_pin;
  float         dist;
} us_sensor_t;

static us_sensor_t us[US_NUM] = {
  { US_C_TRIG_PORT, US_C_TRIG_PIN, US_C_ECHO_PORT, US_C_ECHO_PIN, 999.0f },   /* 正前 */
  { US_L_TRIG_PORT, US_L_TRIG_PIN, US_L_ECHO_PORT, US_L_ECHO_PIN, 999.0f },   /* 左前 */
  { US_R_TRIG_PORT, US_R_TRIG_PIN, US_R_ECHO_PORT, US_R_ECHO_PIN, 999.0f },   /* 右前 */
  { US_LS_TRIG_PORT, US_LS_TRIG_PIN, US_LS_ECHO_PORT, US_LS_ECHO_PIN, 999.0f }, /* 左侧 */
  { US_RS_TRIG_PORT, US_RS_TRIG_PIN, US_RS_ECHO_PORT, US_RS_ECHO_PIN, 999.0f }, /* 右侧 */
};
static uint8_t us_idx = 0;

static float measure(us_sensor_t *s) {
  uint32_t t0;

  /* 触发: 低 2us -> 高 10us -> 低 */
  HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_RESET);
  delay_us(2);
  HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_SET);
  delay_us(10);
  HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_RESET);

  /* 等回波变高 */
  t0 = micros();
  while (HAL_GPIO_ReadPin(s->echo_port, s->echo_pin) == GPIO_PIN_RESET) {
    if (micros() - t0 > US_TIMEOUT_US) {
      return 999.0f;   /* 无回波 */
    }
  }

  /* 测高电平脉宽 */
  t0 = micros();
  while (HAL_GPIO_ReadPin(s->echo_port, s->echo_pin) == GPIO_PIN_SET) {
    if (micros() - t0 > US_TIMEOUT_US) {
      break;
    }
  }
  uint32_t dur = micros() - t0;
  return dur * 0.0343f / 2.0f;   /* 声速约 343m/s, 单程 */
}

void ultrasonic_init(void) {
  for (uint8_t i = 0; i < US_NUM; i++) {
    HAL_GPIO_WritePin(us[i].trig_port, us[i].trig_pin, GPIO_PIN_RESET);
    us[i].dist = 999.0f;
  }
  us_idx = 0;
}

void ultrasonic_update(void) {
  float raw  = measure(&us[us_idx]);
  float prev = us[us_idx].dist;

  /* 限幅滤波 —— 兼顾"抗干扰"与"快速响应":
   *  · 上次无回波(999)     -> 直接接受: 这是新发现的障碍, 不能滤掉
   *  · 突然变近一大截      -> 很可能是干扰毛刺, 限幅(避免误触发避障)
   *  · 距离变大            -> 可能是真越过了障碍, 立即接受(保证及时解除避障) */
  if (prev >= 900.0f) {
    us[us_idx].dist = raw;
  } else if (raw < prev && (prev - raw) > (float)US_MAX_NEAR_JUMP) {
    us[us_idx].dist = prev - (float)US_MAX_NEAR_JUMP;
  } else {
    us[us_idx].dist = raw;
  }

  us_idx = (us_idx + 1) % US_NUM;
}

float ultrasonic_dist(uint8_t idx) {
  return us[idx].dist;
}
