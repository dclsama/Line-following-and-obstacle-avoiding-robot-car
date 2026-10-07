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
  STATE_AVOID,        /* 绕障(三个阶段) */
  STATE_FINISH,       /* 已过终点线, 继续前进让整车越线 */
  STATE_STOP          /* 停车 */
} state_t;

typedef enum {
  AV_TURN_OUT,   /* 转向离开黑线, 让开障碍 */
  AV_PASS,       /* 保持偏角前进, 绕过障碍 */
  AV_BACK        /* 转回朝黑线方向 */
} avoid_sub_t;

static state_t state = STATE_WAIT_START;
static avoid_sub_t avoid_sub = AV_TURN_OUT;
static int8_t  avoid_side = 0;      /* +1=障碍在左(往右绕); -1=障碍在右(往左绕) */

static int32_t snap_l = 0, snap_r = 0;  /* 编码器快照 */
static int8_t last_error = 0;
static uint8_t black_stable = 0;    /* 全黑去抖计数 */
static uint8_t black_latched = 0;   /* 终点线是否已触发 */
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

/* 板载 LED 状态指示(没有串口时靠它看小车在干什么):
 *   灭        = 正常循迹
 *   常亮      = 上电等待
 *   快闪(100ms) = 绕障-转向让开阶段
 *   中闪(250ms) = 绕障-直行绕过阶段
 *   慢闪(500ms) = 绕障-转回找线阶段
 *   慢闪(500ms) = 已到终点/停车 */
static void led_update(void) {
  uint32_t now = HAL_GetTick();

  switch (state) {
    case STATE_WAIT_START:
      LED_ON();
      break;

    case STATE_FOLLOW:
      LED_OFF();
      break;

    case STATE_AVOID: {
      uint32_t period = 500;
      if (avoid_sub == AV_TURN_OUT)    period = 100;
      else if (avoid_sub == AV_PASS)   period = 250;
      if (((now / period) & 1) == 0) { LED_ON(); } else { LED_OFF(); }
      break;
    }

    default:   /* FINISH / STOP: 常亮, 表示"已结束" */
      LED_ON();
      break;
  }
}

#if AVOID_ENABLE
/* 障碍检测(带去抖). 返回 0=无障碍; 否则返回绕行方向: +1=往右绕, -1=往左绕.
 * 赛道上障碍基本出现在【侧前方】, 所以主力是左前/右前这两个斜装传感器;
 * 正前也一并纳入 —— 斜装的两个在正前方存在盲区, 只靠它们会漏检. */
static int8_t obstacle_detect(void) {
  static uint8_t cnt = 0;

  int dc = (int)ultrasonic_dist(US_CENTER);
  int dl = (int)ultrasonic_dist(US_LEFT);
  int dr = (int)ultrasonic_dist(US_RIGHT);
  int ls = (int)ultrasonic_dist(US_LEFT_SIDE);
  int rs = (int)ultrasonic_dist(US_RIGHT_SIDE);

  int lmin = (dl < ls) ? dl : ls;
  int rmin = (dr < rs) ? dr : rs;

  /* 只有"前方/斜前方"的障碍才触发绕行.
     侧方(ls/rs)的障碍不在前进路径上, 车直行撞不到它, 若也触发就会做出
     完全没必要的大动作 —— "墙在正右侧却 180° 调头"就是这么来的. */
  int front_blocked = (dc < OBSTACLE_DIST_CM)
                   || (dl < OBSTACLE_DIST_CM)
                   || (dr < OBSTACLE_DIST_CM);

  if (front_blocked) {
    if (cnt < OBSTACLE_DEBOUNCE) cnt++;
  } else {
    cnt = 0;
  }

  if (cnt >= OBSTACLE_DEBOUNCE) {
    cnt = 0;
    return (rmin >= lmin) ? +1 : -1;    /* 往更空(距离更大)的一侧绕 */
  }
  return 0;
}

/* 绕障转向阶段累计的"左右轮计数差" —— 用来衡量车已经转过多大角度.
 * 弧线转向时 左右轮走过的距离差 ≈ 轮距 × 转角, 所以它能当"转角表"用,
 * 给转向加一道不依赖传感器的硬性上限, 杜绝转过头(比如 180° 调头). */
static int32_t turn_diff(void) {
  int32_t dl = encoder_left() - snap_l;
  int32_t dr = encoder_right() - snap_r;
  int32_t d  = dr - dl;
  return (d < 0) ? -d : d;
}

/* 障碍那一侧的【侧向】是否安全 —— 用于判断"现在可以绕回巡线了".
 *
 * 这里特意只用垂直朝外的侧向传感器, 不用斜前传感器: 车一开始转向, 斜前
 * 传感器的朝向就跟着转, 它的"距离变大"并不代表真的让开了(可能只是转向
 * 把障碍甩出了视野). 侧向传感器测的是垂直方向的净空, 更接近"横向有没有
 * 东西挡着", 正是"能不能安全转回去"该看的量. */
static int side_safe(void) {
  int d = (avoid_side > 0) ? (int)ultrasonic_dist(US_LEFT_SIDE)
                           : (int)ultrasonic_dist(US_RIGHT_SIDE);
  return (d > SIDE_SAFE_DIST_CM);
}
#endif

void fsm_init(void) {
  dwt_init();
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  /* 注意: TIM_CHANNEL_1==0x00, TIM_CHANNEL_2==0x04, 不能用 | 合并(会只启动CH2).
     必须逐个启动, 或用 TIM_CHANNEL_ALL */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
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
      }
      break;

    case STATE_FOLLOW: {
      int e_line, u, kp;

      /* 终点横线检测(全黑, 去抖): 上电即循迹, 第一次全黑即终点 */
      if (line_all_black()) {
        if (black_stable < BLACK_DEBOUNCE) black_stable++;
        if (black_stable >= BLACK_DEBOUNCE && !black_latched) {
          black_latched = 1;
          state = STATE_FINISH;   /* 继续前进让整车越过终点线后再停 */
          snapshot();
          break;
        }
      } else {
        black_stable = 0;
        black_latched = 0;
      }

#if AVOID_ENABLE
      /* 障碍检测 -> 进入绕障 */
      {
        int8_t side = obstacle_detect();
        if (side != 0) {
          avoid_side = side;
          avoid_sub  = AV_TURN_OUT;
          state = STATE_AVOID;
          snapshot();
          break;
        }
      }
#endif

      /* 循迹 PID.
       * 关键1: 微分项只作用于"循迹误差"本身.
       * 关键2: 非线性增益 —— 小误差用小 Kp(直线稳不震荡), 大误差用大 Kp(急弯跟得住).
       * 关键3: 丢线(直角弯常见)时不再慢慢调, 果断朝上次偏差方向急转找线. */
      e_line = line_error();

      if (line_all_white()) {
        u = (last_error >= 0) ? LINE_LOST_U : (-LINE_LOST_U);
      } else {
        kp = (e_line > 1 || e_line < -1) ? LINE_KP_BIG : LINE_KP;
        u = kp * e_line + LINE_KD * (e_line - last_error);
        last_error = (int8_t)e_line;
      }

      motor_set(BASE_PWM - u, BASE_PWM + u);
      break;
    }

#if AVOID_ENABLE
    case STATE_AVOID:
      switch (avoid_sub) {
        case AV_TURN_OUT:
          /* 弧线转向: 一侧慢一侧快, 边走边转 —— 只有弧线才产生真实横向位移,
             原地转只改朝向, 会让"让开了多少"失去参照(旧版 180° 调头的根因).
             退出只看偏角(计数差), 不用超声波 —— 转向中传感器朝向一直在变,
             距离读数不可靠, 这正是之前判断失效的地方. */
          motor_set(AV_TURN_PWM + avoid_side * AV_TURN_SIGN * AV_TURN_DIFF,
                    AV_TURN_PWM - avoid_side * AV_TURN_SIGN * AV_TURN_DIFF);
          if (turn_diff() >= AV_TURN_MAX_DIFF) {
            avoid_sub = AV_PASS;
            snapshot();
          }
          break;

        case AV_PASS:
          /* 弧线绕过: 前进的同时加一个【小的】转向差速, 轨迹是一条外凸的弧线,
             比纯直线自然. 横移量 ≈ 走过的距离 × sin(偏角), 所以这段距离是
             绕开障碍的关键.
             退出: 走够最短距离且侧向安全; 兜底: 距离上限 或 偏角上限
             (弧线会持续累积偏角, 没有偏角上限的话车头会越绕越横). */
          motor_set(AV_PASS_PWM + avoid_side * AV_TURN_SIGN * AV_PASS_DIFF,
                    AV_PASS_PWM - avoid_side * AV_TURN_SIGN * AV_PASS_DIFF);
          if ((avg_abs_travel() >= AV_PASS_MIN_CNT && side_safe())
              || avg_abs_travel() >= AV_PASS_MAX_CNT
              || turn_diff() >= AV_PASS_MAX_DIFF) {
            avoid_sub = AV_BACK;
            snapshot();
          }
          break;

        case AV_BACK:
          /* 反向弧线转回朝黑线. 两个退出条件, 都不需要标定"要转回多少度":
             1) 线回到视野         -> 立刻交给循迹接管
             2) 转够 AV_BACK_MAX_DIFF 仍看不到线 -> 强制交还, 由循迹的丢线逻辑
                自己去把线找回来. 有了它, 整个过程不可能"转不停". */
          if (line_any_black()) {
            /* 线回到视野: 正常交还, last_error 清零让 PID 干净接管 */
            state = STATE_FOLLOW;
            last_error = 0;
          } else if (turn_diff() >= AV_BACK_MAX_DIFF) {
            /* 兜底: 转够了还是看不到线.
               这里 last_error 千万不能清零 —— 交还后循迹处于"全白丢线"状态,
               丢线逻辑是  u = (last_error >= 0) ? +LINE_LOST_U : -LINE_LOST_U,
               清零会让它固定朝一个方向急转(1600的转向量, 又快又错), 越转越远.
               给一个"朝黑线方向"的提示: 往哪边绕开的, 就往反方向找回来. */
            state = STATE_FOLLOW;
            last_error = (int8_t)avoid_side;
          } else {
            /* 用【独立的】平缓参数: 差速小 => 半径大 => 转得缓, 不会像原地打转 */
            motor_set(AV_BACK_PWM - avoid_side * AV_TURN_SIGN * AV_BACK_DIFF,
                      AV_BACK_PWM + avoid_side * AV_TURN_SIGN * AV_BACK_DIFF);
          }
          break;
      }
      break;
#endif

    case STATE_FINISH:
      /* 已检测到终点线, 继续直行 FINISH_EXTRA_CM 让车身完全越线 */
      motor_set(FINISH_PWM, FINISH_PWM);
      if (avg_abs_travel() >= FINISH_EXTRA_CNT) {
        state = STATE_STOP;
        motor_brake();
      }
      break;

    case STATE_STOP:
      motor_brake();
      break;

#if !AVOID_ENABLE
    case STATE_AVOID:   /* 避障关闭时该状态不可达, 列出以保持枚举完整 */
      break;
#endif
  }

  led_update();

#if DEBUG_PRINT
  {
    static uint32_t dbg = 0;
    if (HAL_GetTick() - dbg >= 500) {
      dbg = HAL_GetTick();
      uint8_t lr = line_raw();
      printf("st=%d sub=%d side=%2d encL=%5ld encR=%5ld pwm=%4ld/%4ld line=%d%d%d%d err=%2d us=%d %d %d %d %d\r\n",
             (int)state, (int)avoid_sub, (int)avoid_side,
             (long)encoder_left(), (long)encoder_right(),
             (long)motor_left_out(), (long)motor_right_out(),
             (lr >> 3) & 1, (lr >> 2) & 1, (lr >> 1) & 1, lr & 1,
             (int)line_error(),
             (int)ultrasonic_dist(US_CENTER), (int)ultrasonic_dist(US_LEFT),
             (int)ultrasonic_dist(US_RIGHT),
             (int)ultrasonic_dist(US_LEFT_SIDE), (int)ultrasonic_dist(US_RIGHT_SIDE));
    }
  }
#endif
}
