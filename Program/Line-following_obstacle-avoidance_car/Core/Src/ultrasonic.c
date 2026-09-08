#include "ultrasonic.h"
#include "board.h"
#include "utils.h"

typedef struct {
  GPIO_TypeDef *trig_port;
  uint16_t      trig_pin;
  GPIO_TypeDef *echo_port;
  uint16_t      echo_pin;
  float         dist;
} us_sensor_t;

static us_sensor_t us[3] = {
  { US_C_TRIG_PORT, US_C_TRIG_PIN, US_C_ECHO_PORT, US_C_ECHO_PIN, 999.0f },
  { US_L_TRIG_PORT, US_L_TRIG_PIN, US_L_ECHO_PORT, US_L_ECHO_PIN, 999.0f },
  { US_R_TRIG_PORT, US_R_TRIG_PIN, US_R_ECHO_PORT, US_R_ECHO_PIN, 999.0f },
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
  for (uint8_t i = 0; i < 3; i++) {
    HAL_GPIO_WritePin(us[i].trig_port, us[i].trig_pin, GPIO_PIN_RESET);
    us[i].dist = 999.0f;
  }
  us_idx = 0;
}

void ultrasonic_update(void) {
  us[us_idx].dist = measure(&us[us_idx]);
  us_idx = (us_idx + 1) % 3;
}

float ultrasonic_dist(uint8_t idx) {
  return us[idx].dist;
}
