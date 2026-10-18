#include "ultrasonic.h"
#include "board.h"
#include "utils.h"

/* 轮询 5 路: 前向 3 路用于触发, 侧向 2 路用于绕过阶段判断"障碍是否已经到身后". */
#define US_NUM    5
#define US_INVALID 999.0f    /* "无回波/读数无效"的表示值 */

typedef struct {
  GPIO_TypeDef *trig_port;
  uint16_t      trig_pin;
  GPIO_TypeDef *echo_port;
  uint16_t      echo_pin;
  float         dist;        /* 对外发布的距离(cm), 无效为 999 */
  float         last_raw;    /* 最近一次原始测量值, 仅供调试 */
  float         last_valid;  /* 最近一次有效读数(回波保持用) */
  uint32_t      valid_tick;  /* 那次有效读数的时刻 */
  uint8_t       has_valid;   /* last_valid 是否可用 */
} us_sensor_t;

static us_sensor_t us[US_NUM] = {
  { US_C_TRIG_PORT,  US_C_TRIG_PIN,  US_C_ECHO_PORT,  US_C_ECHO_PIN,  US_INVALID, US_INVALID },
  { US_L_TRIG_PORT,  US_L_TRIG_PIN,  US_L_ECHO_PORT,  US_L_ECHO_PIN,  US_INVALID, US_INVALID },
  { US_R_TRIG_PORT,  US_R_TRIG_PIN,  US_R_ECHO_PORT,  US_R_ECHO_PIN,  US_INVALID, US_INVALID },
  { US_LS_TRIG_PORT, US_LS_TRIG_PIN, US_LS_ECHO_PORT, US_LS_ECHO_PIN, US_INVALID, US_INVALID },
  { US_RS_TRIG_PORT, US_RS_TRIG_PIN, US_RS_ECHO_PORT, US_RS_ECHO_PIN, US_INVALID, US_INVALID },
};
static uint8_t us_idx = 0;

static float measure(us_sensor_t *s) {
  uint32_t t0;

  /* 触发前先看 ECHO 的空闲电平.
   * HC-SR04 空闲时会把 ECHO 主动拉低; 如果这里读到高电平, 说明这一路
   * 悬空/被外部上拉(左前接在 PA12 = 蓝板 USB D+, 自带 1.5k 上拉),
   * 此时测出来的脉宽全是噪声, 直接判无效, 不参与避障判定. */
  if (HAL_GPIO_ReadPin(s->echo_port, s->echo_pin) == GPIO_PIN_SET) {
    return US_INVALID;
  }

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
      return US_INVALID;   /* 无回波 */
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
    us[i].dist       = US_INVALID;
    us[i].last_raw   = US_INVALID;
    us[i].last_valid = US_INVALID;
    us[i].valid_tick = 0;
    us[i].has_valid  = 0;
  }
  us_idx = 0;
}

void ultrasonic_update(void) {
  us_sensor_t *s = &us[us_idx];
  float raw = measure(s);

  /* last_raw 保留【未经过窗口】的原始值, 用来区分两种失败:
   *   999 = 连回波上升沿都没等到   |   >US_MAX_VALID_CM = 有回波但超量程 */
  s->last_raw = raw;

  /* 有效性窗口: 盲区内的短脉冲、超时截断、ECHO 被上拉悬空, 一律当"无回波". */
  if (raw < (float)US_BLIND_CM || raw > (float)US_MAX_VALID_CM) {
    /* ---- 回波保持 ----
     * 斜角摆放的障碍经常"越靠近越测不到"(声束被反射走). 这里在丢失回波后
     * 的 US_HOLD_MS 内沿用上一次有效读数, 把接近过程中的丢包补上.
     * 保持时间刻意短于撞上判定(200ms), 所以不会把"刚离开视野"误判成顶住. */
    if (s->has_valid && (HAL_GetTick() - s->valid_tick) <= US_HOLD_MS) {
      s->dist = s->last_valid;
    } else {
      s->dist      = US_INVALID;
      s->has_valid = 0;
    }
  } else {
    s->dist       = raw;
    s->last_valid = raw;
    s->valid_tick = HAL_GetTick();
    s->has_valid  = 1;
  }

  us_idx = (uint8_t)((us_idx + 1) % US_NUM);
}

float ultrasonic_dist(uint8_t idx) {
  return us[idx].dist;
}

float ultrasonic_last_raw(uint8_t idx) {
  return us[idx].last_raw;
}
