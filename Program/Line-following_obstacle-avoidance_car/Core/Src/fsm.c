#include "fsm.h"
#include "board.h"
#include "utils.h"
#include "motor.h"
#include "encoder.h"
#include "line.h"
#include "ultrasonic.h"
#include "tim.h"
#include <stdio.h>

typedef enum {
  STATE_WAIT_START,   /* 上电等待 */
  STATE_FOLLOW,       /* 循迹 */
  STATE_AVOID,        /* 绕障 */
  STATE_REACQUIRE,    /* 重新找线 */
  STATE_STOP          /* 停车 */
} state_t;

typedef enum {
  AVOID_TURN_AWAY,    /* 转向远离障碍(约90°) */
  AVOID_STRAIGHT,     /* 直行绕过 */
  AVOID_TURN_BACK     /* 转回朝线方向 */
} avoid_sub_t;

static state_t state = STATE_WAIT_START;
static avoid_sub_t avoid_sub = AVOID_TURN_AWAY;
static int8_t avoid_dir = 1;        /* +1 向右绕, -1 向左绕 */

static int32_t snap_l = 0, snap_r = 0;  /* 编码器快照 */
static int8_t last_error = 0;
static uint8_t black_stable = 0;    /* 全黑去抖计数 */
static uint8_t black_latched = 0;   /* 本根横线是否已计数 */
static uint8_t black_events = 0;    /* 全黑次数(起终点) */
static uint8_t obs_debounce = 0;    /* 障碍去抖计数 */
static uint32_t state_tick = 0;

static void snapshot(void) {
  snap_l = encoder_left();
  snap_r = encoder_right();
}

static int32_t avg_abs_travel(void) {
  int32_t dl = encoder_left() - snap_l;
  int32_t dr = encoder_right() - snap_r;
  if (dl < 0) dl = -dl;
  if (dr < 0) dr = -dr;
  return (dl + dr) / 2;
}

void fsm_init(void) {
  dwt_init();
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1 | TIM_CHANNEL_2);
  encoder_init();
  motor_init();
  line_init();
  ultrasonic_init();
  state = STATE_WAIT_START;
  state_tick = HAL_GetTick();
}

void fsm_run(void) {
  static uint32_t last = 0;
  if (HAL_GetTick() - last < LOOP_MS) return;
  last = HAL_GetTick();

  encoder_update();
  line_update();
  ultrasonic_update();

  switch (state) {
    case STATE_WAIT_START:
      motor_stop();
      if (HAL_GetTick() - state_tick >= STARTUP_DELAY_MS) {
        state = STATE_FOLLOW;
        last_error = 0;
        black_stable = 0;
        black_latched = 0;
        black_events = 0;
        obs_debounce = 0;
      }
      break;

    case STATE_FOLLOW: {
      int dc, dl, dr, err, u;

      /* 起终点横线检测(全黑) */
      if (line_all_black()) {
        if (black_stable < BLACK_DEBOUNCE) black_stable++;
        if (black_stable == BLACK_DEBOUNCE && !black_latched) {
          black_latched = 1;
          black_events++;
          if (black_events >= BLACK_EVENTS_STOP) {
            state = STATE_STOP;
            motor_brake();
            break;
          }
        }
      } else {
        black_stable = 0;
        black_latched = 0;
      }

      /* 障碍检测(去抖) */
      dc = (int)ultrasonic_dist(US_CENTER);
      dl = (int)ultrasonic_dist(US_LEFT);
      dr = (int)ultrasonic_dist(US_RIGHT);
      if (dc < OBSTACLE_DIST_CM || dl < OBSTACLE_DIST_CM || dr < OBSTACLE_DIST_CM) {
        if (obs_debounce < OBSTACLE_DEBOUNCE) obs_debounce++;
      } else {
        obs_debounce = 0;
      }
      if (obs_debounce >= OBSTACLE_DEBOUNCE) {
        avoid_dir = (dl <= dr) ? 1 : -1;   /* 左近->向右绕, 右近->向左绕 */
        state = STATE_AVOID;
        avoid_sub = AVOID_TURN_AWAY;
        snapshot();
        break;
      }

      /* 循迹 PID */
      err = line_error();
      if (line_all_white()) err = last_error;   /* 丢线保持上次误差 */
      u = LINE_KP * err + LINE_KD * (err - last_error);
      last_error = (int8_t)err;
      motor_set(BASE_PWM - u, BASE_PWM + u);
      break;
    }

    case STATE_AVOID:
      switch (avoid_sub) {
        case AVOID_TURN_AWAY:
          motor_set(avoid_dir * TURN_PWM, -avoid_dir * TURN_PWM);
          if (avg_abs_travel() >= AVOID_TURN_CNT) {
            avoid_sub = AVOID_STRAIGHT;
            snapshot();
          }
          break;
        case AVOID_STRAIGHT:
          motor_set(STRAIGHT_PWM, STRAIGHT_PWM);
          if (avg_abs_travel() >= AVOID_STRAIGHT_CNT) {
            avoid_sub = AVOID_TURN_BACK;
            snapshot();
          }
          break;
        case AVOID_TURN_BACK:
          motor_set(-avoid_dir * TURN_PWM, avoid_dir * TURN_PWM);
          if (avg_abs_travel() >= AVOID_TURN_CNT) {
            state = STATE_REACQUIRE;
            snapshot();
          }
          break;
      }
      break;

    case STATE_REACQUIRE:
      /* 缓慢向线一侧转弯前进, 找线 */
      motor_set(REACQUIRE_PWM - avoid_dir * REACQUIRE_STEER,
                REACQUIRE_PWM + avoid_dir * REACQUIRE_STEER);
      if (line_any_black()) {
        state = STATE_FOLLOW;
        last_error = 0;
      } else if (avg_abs_travel() >= REACQUIRE_MAX_CNT) {
        state = STATE_STOP;   /* 找不到线, 安全停车 */
        motor_brake();
      }
      break;

    case STATE_STOP:
      motor_brake();
      break;
  }

#if DEBUG_PRINT
  {
    static uint32_t dbg = 0;
    if (HAL_GetTick() - dbg >= 500) {
      dbg = HAL_GetTick();
      uint8_t lr = line_raw();
      printf("st=%d encL=%5ld encR=%5ld line=%d%d%d%d err=%2d us=%d %d %d\r\n",
             (int)state, (long)encoder_left(), (long)encoder_right(),
             (lr >> 3) & 1, (lr >> 2) & 1, (lr >> 1) & 1, lr & 1,
             (int)line_error(),
             (int)ultrasonic_dist(US_CENTER), (int)ultrasonic_dist(US_LEFT),
             (int)ultrasonic_dist(US_RIGHT));
    }
  }
#endif
}
