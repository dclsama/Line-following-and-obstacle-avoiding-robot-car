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
  STATE_PUSH,         /* 顶住障碍物: 直接向前推开再继续 */
  STATE_FINISH,       /* 已过终点线, 继续前进让整车越线 */
  STATE_STOP          /* 停车 */
} state_t;

typedef enum {
  AV_TURN_OUT,   /* ① 原地转 θ: 车头斜指赛道外侧 */
  AV_SLANT_OUT,  /* ② 斜出段 L1: 边走边让开 */
  AV_TURN_PAR,   /* ③ 原地反向转 θ: 车头回到与黑线平行 */
  AV_PARALLEL,   /* ④ 平行段 L2: 车身平行地从障碍旁边通过 */
  AV_TURN_IN,    /* ⑤ 原地反向转 θ: 车头斜指黑线 */
  AV_SEARCH,     /* ⑥ 直行找线, 压到线后交还循迹 */
  AV_SEARCH_TURN /* ⑥ 找线 2s 未果: 朝黑线再转 30° 后继续找(只追加一次) */
} avoid_sub_t;

static state_t state = STATE_WAIT_START;
static avoid_sub_t avoid_sub = AV_TURN_OUT;

static int32_t snap_l = 0, snap_r = 0;  /* 编码器快照 */
static int8_t last_error = 0;
static int16_t u_prev = 0;          /* 上一周期的转向量(用于斜率限制) */
static int16_t e_filt_q4 = 0;       /* 低通滤波后的循迹误差, 放大16倍存(定点) */
static uint8_t black_stable = 0;    /* 全黑去抖计数 */
static uint8_t black_latched = 0;   /* 当前这根横线是否已处理 */
static int32_t travel_cnt = 0;      /* 从起步开始累计的里程(编码器计数) */
static int32_t last_enc_l = 0, last_enc_r = 0;
static uint32_t state_tick = 0;
static uint32_t finish_guard_until = 0;   /* 这段时间内不做终点检测(绕障前后) */
static uint8_t  rejoin_needed = 0;        /* 1 = 绕障后还没回到线上, 暂不接受新避障 */
static uint32_t avoid_stage_tick = 0;     /* 当前避障子阶段的开始时刻(超时兜底/LED 用) */
static uint32_t stall_tick = 0;           /* 堵转检测窗口起点 */
static int32_t  stall_ref  = 0;           /* 窗口起点时的 travel_cnt */
#if AVOID_ENABLE
static uint32_t rejoin_deadline = 0;      /* 回线的兜底期限 */
#endif

/* 当前有几路传感器压黑线(0~4) */
static uint8_t line_black_count(void) {
  uint8_t lr = line_raw();
  uint8_t n = 0;
  if (lr & 0x08) n++;   /* 左远 */
  if (lr & 0x04) n++;   /* 左近 */
  if (lr & 0x02) n++;   /* 右近 */
  if (lr & 0x01) n++;   /* 右远 */
  return n;
}

static void snapshot(void) {
  snap_l = encoder_left();
  snap_r = encoder_right();
}

/* 终点线检测(横线 + 里程 + 去抖). 返回 1 表示已切换到 STATE_FINISH.
 * 抽成函数是为了让 AV_BACK 阶段也能检测 —— 否则避障过程中压过终点线
 * 这个 30ms 窗口不会被记录, 车会越过终点继续跑. */
static uint8_t check_finish_line(void) {
  uint8_t cnt;

  /* 绕障过程中, 以及交还后的一小段时间内, 不做终点判定.
   * 车斜着绕回黑线时车身与线接近垂直, 会有 3~4 路同时压线 ——
   * 正好符合"3 路以上 = 横线"的终点判据, 导致绕回途中误停车. */
  if (state == STATE_AVOID || rejoin_needed
      || HAL_GetTick() < finish_guard_until) {
    black_stable = 0;
    black_latched = 0;
    return 0;
  }

  cnt = line_black_count();

  /* 判定"横线"必须【至少 3 路同时压线】.
   *
   * 为什么不能放宽到 2 路: 循迹时车本来就骑在线上, 4 路传感器中间相邻的
   * 两路经常同时压线 —— "2 路就算横线"等于"一直算横线". 跑出 1 米
   * (FINISH_MIN_TRAVEL) 之后就会随机停车, 而且是 LED 常亮进 STOP.
   *
   * 3 路的道理: 黑线只有约 2cm 宽, 正常循迹最多同时压住 2 路相邻传感器;
   * 而终点是垂直于行进方向的横线, 车身斜着压上去时会同时盖住 3~4 路. */
  uint8_t hit = (cnt >= 3);

  if (hit) {
    if (black_stable < BLACK_DEBOUNCE) black_stable++;
    if (black_stable >= BLACK_DEBOUNCE && !black_latched
        && travel_cnt >= FINISH_MIN_TRAVEL_CNT) {
      black_latched = 1;
      state = STATE_FINISH;
      snapshot();
      return 1;
    }
  } else {
    black_stable = 0;
    black_latched = 0;
  }
  return 0;
}

static int32_t avg_abs_travel(void) {
  int32_t dl = encoder_left() - snap_l;
  int32_t dr = encoder_right() - snap_r;
  if (dl < 0) dl = -dl;
  if (dr < 0) dr = -dr;
  return (dl + dr) / 2;
}

/* ---- 撞上障碍物判定 ----
 * 前向三路里最近的那一路持续读到极近距离 -> 认为已经顶上了. */
static uint32_t collide_since = 0;
static uint8_t  collide_arm = 0;

static int front_nearest(void) {
  int d = (int)ultrasonic_dist(US_CENTER);
  int t = (int)ultrasonic_dist(US_LEFT);
  if (t < d) d = t;
  t = (int)ultrasonic_dist(US_RIGHT);
  if (t < d) d = t;
  return d;
}

static uint8_t collision_detected(void) {
  if (front_nearest() >= US_COLLISION_CM) {
    collide_arm = 0;
    return 0;
  }
  if (!collide_arm) {
    collide_arm   = 1;
    collide_since = HAL_GetTick();
    return 0;
  }
  if (HAL_GetTick() - collide_since < COLLISION_CONFIRM_MS) return 0;
  return 1;
}

/* 板载 LED 状态指示(没有串口时靠它看小车在干什么):
 *   灭          = 正常循迹
 *   50ms 快闪   = 循迹中测到障碍(见 STATE_FOLLOW 分支)
 *   100ms 快闪  = 绕障-转向让开阶段
 *   250ms 中闪  = 绕障-直行绕过阶段
 *   500ms 慢闪  = 绕障-转回找线阶段
 *   1000ms 慢闪 = 顶开障碍物阶段
 *   常亮        = 上电等待 / 已到终点或停车 */
static void led_update(void) {
  uint32_t now = HAL_GetTick();

  switch (state) {
    case STATE_WAIT_START:
      LED_ON();
      break;

    case STATE_FOLLOW:
      /* 循迹状态下的三档指示, 没有串口时靠它定位问题在哪一层:
       *   50ms 快闪  = 前方/斜前测到的距离已进入触发范围(但可能被冷却挡住)
       *   200ms 慢闪 = 收到了有效回波, 但距离还没到触发范围(传感器工作正常)
       *   灭         = 三路全都没有有效回波(不是阈值问题, 是根本没测到) */
#if AVOID_ENABLE
      {
        float dc = ultrasonic_dist(US_CENTER);
        float dl = ultrasonic_dist(US_LEFT);
        float dr = ultrasonic_dist(US_RIGHT);
        float dmin = dc;
        if (dl < dmin) dmin = dl;
        if (dr < dmin) dmin = dr;

        /* 原始值(未经过量程窗口): 只要不是 999, 就说明收到过回波 */
        float rc = ultrasonic_last_raw(US_CENTER);
        float rl = ultrasonic_last_raw(US_LEFT);
        float rr = ultrasonic_last_raw(US_RIGHT);
        uint8_t any_echo = (rc < 900.0f || rl < 900.0f || rr < 900.0f);

        if (dc < (float)OBSTACLE_DIST_CM
            || dl < (float)OBSTACLE_DIST_L_CM
            || dr < (float)OBSTACLE_DIST_R_CM) {
          if (((now / 50) & 1) == 0) { LED_ON(); } else { LED_OFF(); }   /* 快闪: 该避障了 */
        } else if (dmin < 900.0f) {
          if (((now / 200) & 1) == 0) { LED_ON(); } else { LED_OFF(); }  /* 慢闪: 测到了但还远 */
        } else if (any_echo) {
          LED_ON();                                                     /* 常亮: 有回波但超量程 */
        } else {
          LED_OFF();                                                    /* 灭: 完全没有回波 */
        }
      }
#else
      LED_OFF();
#endif
      break;

    case STATE_AVOID: {
      /* 用"同一组里闪几下"表示当前是第几段, 每换一段重新开始数:
       *   1 下 = ① 原地转出      2 下 = ② 斜出
       *   3 下 = ③ 转回平行      4 下 = ④ 平行通过
       *   5 下 = ⑤ 转向黑线      6 下 = ⑥ 直行找线
       * 亮 130ms / 灭 170ms 算"一下", 一组数完灭 900ms 再重复.
       * 用 avoid_stage_tick 做基准, 所以每次阶段切换都会从头开始闪. */
      uint32_t n, t, cycle;
      switch (avoid_sub) {
        case AV_TURN_OUT:  n = 1; break;
        case AV_SLANT_OUT: n = 2; break;
        case AV_TURN_PAR:  n = 3; break;
        case AV_PARALLEL:  n = 4; break;
        case AV_TURN_IN:   n = 5; break;
        default:           n = 6; break;   /* AV_SEARCH */
      }
      t = now - avoid_stage_tick;
      cycle = n * 300u + 900u;
      t %= cycle;
      if (t < n * 300u && (t % 300u) < 130u) {
        LED_ON();
      } else {
        LED_OFF();
      }
      break;
    }

    case STATE_PUSH:
      /* 1000ms 慢闪(1Hz) = 正在顶开障碍物 */
      if (((now / 1000) & 1) == 0) { LED_ON(); } else { LED_OFF(); }
      break;

    default:   /* FINISH / STOP: 常亮, 表示"已结束" */
      LED_ON();
      break;
  }
}

#if AVOID_ENABLE
/* 绕障冷却: 刚绕过障碍后的短时间内, 忽略【同一侧】的障碍,
 * 避免把"刚刚绕过去的那个障碍"当成新障碍, 交还循迹后立刻又触发一次. */
static int8_t   avoid_side = 0;             /* +1=障碍在左(往右绕); -1=障碍在右(往左绕) */
static uint32_t avoid_cooldown_until = 0;   /* 同侧冷却截止时刻 */
static uint32_t avoid_settle_until = 0;     /* 绕障刚结束的静默期截止时刻(不分左右) */
static int8_t   avoid_last_side = 0;        /* 上一次绕障的方向 */
static uint8_t  search_retry_done = 0;      /* 找线阶段的"追加转向"是否已经用过 */

/* 障碍检测. 返回 0=无障碍; 否则返回绕行方向: +1=往右绕, -1=往左绕.
 *
 * 方向判定原则改成"谁看见的听谁的":
 *   · 只有左前看见 -> 障碍在左 -> 往右绕(+1)
 *   · 只有右前看见 -> 障碍在右 -> 往左绕(-1)
 *   · 两边都看见   -> 往距离更大(更空)的一侧绕
 *   · 只有正前看见 -> 用两侧的侧向净空决定
 * 原来一律拿 min(dl,ls) 和 min(dr,rs) 比, 侧向传感器读数一偏就可能把方向
 * 判反, 甚至朝障碍那一侧绕. */
static int8_t obstacle_detect(void) {
  /* 命中历史: 每 10ms 一拍, bit0 = 本拍, 越往高位越早.
   * 确认条件改成"窗口内累计命中 OBSTACLE_WINDOW_HITS 拍", 不再要求连续命中 ——
   * 斜角障碍的回波本来就是断断续续的, 连续判据一丢就清零, 永远凑不满. */
  static uint32_t hit_hist = 0;
  static uint32_t last_call = 0;       /* 上次调用的时刻: 检测没在跑时要清历史 */
  static int16_t  side_vote = 0;       /* 方向投票: >0=障碍偏左(往右绕), <0=偏右 */
  static int16_t  near_score = 0;      /* 近距带的持续计数: 命中+1, 空拍-1 */

  int dc = (int)ultrasonic_dist(US_CENTER);
  int dl = (int)ultrasonic_dist(US_LEFT);
  int dr = (int)ultrasonic_dist(US_RIGHT);

  int left_hit   = (dl < OBSTACLE_DIST_L_CM);
  int right_hit  = (dr < OBSTACLE_DIST_R_CM);
  int center_hit = (dc < OBSTACLE_DIST_CM);
  int hit_now    = (center_hit || left_hit || right_hit);

  /* 近距带: 落在阈值外侧但仍在 [阈值, 阈值+余量] 的读数 */
  int left_near   = (dl < OBSTACLE_DIST_L_CM + OBSTACLE_NEAR_MARGIN_CM);
  int right_near  = (dr < OBSTACLE_DIST_R_CM + OBSTACLE_NEAR_MARGIN_CM);
  int center_near = (dc < OBSTACLE_DIST_CM + OBSTACLE_NEAR_MARGIN_CM);
  int near_now    = (center_near || left_near || right_near);

  uint32_t now = HAL_GetTick();
  uint8_t  hits;
  uint32_t window;

  /* 检测中断过(进入避障/静默期/顶开), 旧的历史不能带进来, 否则一恢复就立刻触发 */
  if (now - last_call > 30) {
    hit_hist   = 0;
    side_vote  = 0;
    near_score = 0;
  }
  last_call = now;

  /* 已经贴到极近距离: 绕不过去了, 交给"顶开"逻辑处理, 不再启动绕障 */
  {
    int dmin = dc;
    if (dl < dmin) dmin = dl;
    if (dr < dmin) dmin = dr;
    if (dmin < US_COLLISION_CM) {
      hit_hist   = 0;
      side_vote  = 0;
      near_score = 0;
      return 0;
    }
  }

  /* 只有"前方/斜前方"的障碍才触发绕行. */
  hit_hist = (hit_hist << 1) | (hit_now ? 1u : 0u);

  window = hit_hist & ((1u << OBSTACLE_WINDOW_CYCLES) - 1u);
  hits = 0;
  for (uint8_t i = 0; i < OBSTACLE_WINDOW_CYCLES; i++) {
    if (window & (1u << i)) hits++;
  }

  /* 近距带积分: 持续存在就累加, 离开就慢慢回落 */
  if (near_now) {
    if (near_score < OBSTACLE_NEAR_HITS) near_score++;
  } else if (near_score > 0) {
    near_score--;
  }

  /* ---- 方向判定: 相对远近 + 投票 ----
   * 只在命中拍计票; 没命中的拍让票数缓慢衰减, 避免旧方向一直挂着.
   * 这样可以跨着"回波丢失"继续累计, 方向比原来稳. */
  if (hit_now || near_now) {
    if (dl < 900 && dr < 900) {
      /* 左右前都测到了有效距离: 只有差距足够明确才计票, 否则算"看不清" */
      if (dl + OBSTACLE_SIDE_MARGIN_CM <= dr)      side_vote += 1;
      else if (dr + OBSTACLE_SIDE_MARGIN_CM <= dl) side_vote -= 1;
    } else if (left_near) {
      side_vote += 1;
    } else if (right_near) {
      side_vote -= 1;
    }
  } else {
    if (side_vote > 0) side_vote--;
    else if (side_vote < 0) side_vote++;
  }
  if (side_vote >  100) side_vote =  100;
  if (side_vote < -100) side_vote = -100;

  /* 两条确认路径:
   *   · 阈值内间歇命中  -> 80ms 内确认(正常障碍)
   *   · 阈值外近距带持续存在 -> 约 500ms 延时确认(斜面/远一点的障碍) */
  if (hits < OBSTACLE_WINDOW_HITS && near_score < OBSTACLE_NEAR_HITS) return 0;

  hit_hist   = 0;   /* 确认后清空, 下一次重新累计 */
  near_score = 0;

  {
    int8_t side;
    if (side_vote > 0)      side = +1;                     /* 障碍偏左 -> 往右绕 */
    else if (side_vote < 0) side = -1;                     /* 障碍偏右 -> 往左绕 */
    /* 投票不明确: 用左右前的实际远近; 连这个也分不出来就沿用上次的绕障方向,
     * 避免在同一区域来回切换(翻来覆去). */
    else if (dl != dr)      side = (dl < dr) ? +1 : -1;
    else                    side = (avoid_last_side != 0) ? avoid_last_side : +1;

    side_vote = 0;
    return side;
  }
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
/* 绕障结束、交还循迹的统一出口.
 * line_visible: 交还这一刻黑线是否在视野里 —— 在就用当前误差, 不在就用
 *               "往哪边绕开的就往反方向找"作为提示. */
static void avoid_handover(uint8_t line_visible) {
  state = STATE_FOLLOW;
  last_error = line_visible ? (int8_t)line_error() : (int8_t)avoid_side;
  u_prev = 0;                               /* 防止交还后第一拍猛打舵 */
  e_filt_q4 = (int16_t)(line_error() * 16);
  avoid_last_side = avoid_side;
  avoid_cooldown_until = HAL_GetTick() + AVOID_COOLDOWN_MS;
  avoid_settle_until   = HAL_GetTick() + AVOID_SETTLE_MS;
  finish_guard_until   = HAL_GetTick() + FINISH_GUARD_MS;

  /* 一次绕障流程要等车重新回到线上才算结束. 在此之前不再接受新避障,
   * 否则并线过程中车头一扫到路边的"无关障碍"就会又绕一次. */
  rejoin_needed   = 1;
  rejoin_deadline = HAL_GetTick() + AVOID_REJOIN_TIMEOUT_MS;
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

  /* 累计里程(取左右轮增量的平均绝对值), 用于区分起跑线和终点线 */
  {
    int32_t dl = encoder_left() - last_enc_l;
    int32_t dr = encoder_right() - last_enc_r;
    if (dl < 0) dl = -dl;
    if (dr < 0) dr = -dr;
    travel_cnt += (dl + dr) / 2;

    last_enc_l = encoder_left();
    last_enc_r = encoder_right();
  }

  switch (state) {
    case STATE_WAIT_START:
      motor_stop();
      if (HAL_GetTick() - state_tick >= STARTUP_DELAY_MS) {
        state = STATE_FOLLOW;
        last_error = 0;
        black_stable = 0;
        black_latched = 0;
        /* 里程计数从出发这一刻起算 */
        travel_cnt = 0;
        last_enc_l = encoder_left();
        last_enc_r = encoder_right();
        stall_ref  = 0;
        stall_tick = HAL_GetTick();
      }
      break;

    case STATE_FOLLOW: {
      int e_line, u, kp, spd;

      /* 终点线检测(横线 + 里程 + 去抖), 详见 check_finish_line() */
      if (check_finish_line()) break;

      /* 已经顶到障碍物上 -> 直接推开, 不再尝试绕行 */
      if (collision_detected()) {
        state = STATE_PUSH;
        state_tick = HAL_GetTick();
        collide_arm = 0;
        break;
      }

#if AVOID_ENABLE
      /* 障碍检测 -> 进入绕障.
       * 静默期(AVOID_SETTLE_MS)内任何一侧都不触发: 刚绕完的那个障碍会随着
       * 车头摆动重新进入同一路传感器的视野, 距离和角度跟新障碍一模一样.
       * 静默期过后, 同一侧再冷却 AVOID_COOLDOWN_MS. */
      /* 一次绕障流程要等车回到线上才算结束; 超时兜底防止一直找不到线时永久失明.
       * 刚回线的这一下, 车身往往还是斜着压线的, 再宽限一段时间不做终点判定. */
      if (rejoin_needed && (line_any_black() || HAL_GetTick() >= rejoin_deadline)) {
        rejoin_needed = 0;
        finish_guard_until = HAL_GetTick() + FINISH_GUARD_MS;
      }

      if (!rejoin_needed && HAL_GetTick() >= avoid_settle_until) {
        int8_t side = obstacle_detect();

        if (side != 0
            && (HAL_GetTick() >= avoid_cooldown_until || side != avoid_last_side)) {
          avoid_side = side;
          avoid_sub  = AV_TURN_OUT;
          state = STATE_AVOID;
          avoid_stage_tick = HAL_GetTick();   /* 重置子阶段超时计时 */
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
        /* 丢线(直角弯常见): 朝上次偏差方向转回去找线.
         * 避障刚结束、还没重新压上线时(rejoin_needed)用较柔和的舵量,
         * 避免以大角度冲过线后又要猛修 —— 压上线后立即恢复满舵权限. */
#if AVOID_ENABLE
        if (rejoin_needed) {
          u = (last_error >= 0) ? AVOID_SEARCH_U : (-AVOID_SEARCH_U);
        } else
#endif
        {
          /* 丢线: 朝上次偏差方向找线, 但转向量限到 LINE_LOST_U_MAX,
           * 内轮不停转 -> 画弧搜索, 而不是原地打转. */
          u = (last_error >= 0) ? LINE_LOST_U : (-LINE_LOST_U);
          if (u > LINE_LOST_U_MAX) u = LINE_LOST_U_MAX;
          else if (u < -LINE_LOST_U_MAX) u = -LINE_LOST_U_MAX;
        }
      } else {
        /* 误差低通: 平滑数字传感器的离散跳变(放大16倍做定点运算) */
        {
          int16_t target = (int16_t)(e_line * 16);   /* 用乘法, 避免负左移的实现定义行为 */
          e_filt_q4 += (target - e_filt_q4) >> LINE_E_FILTER_SHIFT;
        }

        kp = (e_line > 1 || e_line < -1) ? LINE_KP_BIG : LINE_KP;
        u  = (kp * e_filt_q4) >> 4;                       /* P 项用滤波后的误差 */
        u += LINE_KD * (e_line - last_error);             /* D 项; KD=0 时不生效 */
        last_error = (int8_t)e_line;
      }

      /* 转向量限幅 —— 丢线分支也必须经过这里!
       * 之前限幅只写在"看到线"的分支里, 而丢线时 LINE_LOST_U(1270) >
       * BASE_PWM(900), 内轮会反转(-370), 正是"丢线甩尾"的来源.
       * 限在这里: 内轮最多减速到停, 不允许反转. */
      if (u > LINE_U_MAX) u = LINE_U_MAX;
      else if (u < -LINE_U_MAX) u = -LINE_U_MAX;

      /* 转向量斜率限制: 抑制左右晃动.
       * 误差是离散跳变的(只在整数间变), PID 输出会跟着一跳一跳, 车就摆.
       * 限制每周期变化量, 相当于给执行器加惯性; 稳态时 u 不变化, 精度不受影响. */
      {
        /* 分级: 小误差时收紧(甩动主要发生在线上小幅来回), 大误差时放宽(弯道要响应).
         * 丢线时按大误差处理, 否则要 13 个周期(130ms)才转到位, 车会冲过弯. */
        int slew = (e_line >= 2 || e_line <= -2 || line_all_white())
                   ? LINE_U_SLEW_BIG : LINE_U_SLEW;
        int du = u - u_prev;
        if (du > slew) du = slew;
        else if (du < -slew) du = -slew;
        u = u_prev + du;
        u_prev = u;
      }

      /* 预警减速: 障碍进入预警区就先降速(还不避障), 给避障动作留出距离.
       * 转向量同比例缩放, 保证轨迹形状不变, 只是整体变慢. */
      spd = BASE_PWM;
#if AVOID_ENABLE
      {
        int dc = (int)ultrasonic_dist(US_CENTER);
        int dl = (int)ultrasonic_dist(US_LEFT);
        int dr = (int)ultrasonic_dist(US_RIGHT);
        int nearest = dc;
        if (dl < nearest) nearest = dl;
        if (dr < nearest) nearest = dr;

        if (nearest < OBSTACLE_WARN_CM) {
          spd = BASE_PWM * OBSTACLE_WARN_PCT / 100;
          u   = u * OBSTACLE_WARN_PCT / 100;
        }
      }
#endif
      /* ---- 编码器堵转检测(超声波漏检的兜底) ----
       * 有些障碍物摆放角度会把声束反射走, 超声波从头到尾读"无回波",
       * 阈值再大也没用. 这一层不看传感器, 只看车有没有真的在走. */
      if (HAL_GetTick() - stall_tick >= STALL_WINDOW_MS) {
        int32_t moved = travel_cnt - stall_ref;
        stall_ref  = travel_cnt;
        stall_tick = HAL_GetTick();

        if (moved < 0) moved = 0;   /* travel_cnt 被重置时的保护 */
        if (spd >= STALL_MIN_PWM && moved < STALL_MIN_CNT) {
          state = STATE_PUSH;       /* 顶住了: 直接向前推开 */
          state_tick = HAL_GetTick();
          collide_arm = 0;
          break;
        }
      }

      motor_set(spd - u, spd + u);
      break;
    }

#if AVOID_ENABLE
    case STATE_AVOID:
      /* 绕障流程一旦开始就【按固定动作走完】, 中途不再接受任何新的触发.
       *
       * 之前这里还挂着"顶到东西就转顶开"的检测, 结果是: 车转向/绕过时
       * 传感器扫到赛道旁边那些"本来碰不到"的障碍, 就会中途改动作.
       * 现在整段流程只由三个阶段自己的角度/距离判据推进. */
      switch (avoid_sub) {
        case AV_TURN_OUT:
          /* ① 原地转出 θ: 车头斜指赛道外侧. 纯打转, 不产生位移.
           * 退出只看偏角(左右轮计数差), 不用超声波 —— 转向中传感器朝向
           * 一直在变, 距离读数不可靠. */
          motor_set(avoid_side * AV_PIVOT_PWM, -avoid_side * AV_PIVOT_PWM);
          if (turn_diff() >= AV_TURN_MAX_DIFF
              || HAL_GetTick() - avoid_stage_tick > AV_STAGE_TIMEOUT_MS) {
            avoid_sub = AV_SLANT_OUT;
            avoid_stage_tick = HAL_GetTick();
            snapshot();
          }
          break;

        case AV_SLANT_OUT:
          /* ② 斜出段: 保持 θ 角直行, 横移 = 段长 × sinθ.
           * 退出只看距离 —— 这一段本来就不该转向, turn_diff 的增长只可能是
           * 单轮打滑, 用它做判据会把直行变成原地打转. */
          motor_set(AV_PASS_PWM, AV_PASS_PWM);
          if (avg_abs_travel() >= AV_SLANT_OUT_CNT
              || HAL_GetTick() - avoid_stage_tick > AV_STAGE_TIMEOUT_MS) {
            avoid_sub = AV_TURN_PAR;
            avoid_stage_tick = HAL_GetTick();
            snapshot();
          }
          break;

        case AV_TURN_PAR:
          /* ③ 反向转: 角度独立, 比转出角小一点, 车身略朝外侧进入平行段.
           *
           * 找到线就优先循迹: 从这一段开始车已经离开黑线, 只要任意一路
           * 重新看到黑线, 立刻中止剩余动作交给循迹 —— 不再"走完固定流程".
           * (① 和 ② 不能加这个判断: 那时车还压在线上, 一判断就会立刻退出.) */
          if (line_any_black()) { avoid_handover(1); break; }

          motor_set(-avoid_side * AV_PIVOT_PWM, avoid_side * AV_PIVOT_PWM);
          if (turn_diff() >= AV_TURN_PAR_DIFF
              || HAL_GetTick() - avoid_stage_tick > AV_STAGE_TIMEOUT_MS) {
            avoid_sub = AV_PARALLEL;
            avoid_stage_tick = HAL_GetTick();
            snapshot();
          }
          break;

        case AV_PARALLEL:
          /* ④ 平行段: 车身与黑线平行地从障碍旁边通过.
           * 这一段决定"车尾是否已经完全越过障碍", 太短会刮到后轮.
           * 一旦重新看到黑线(比如赛道弯回来), 直接交给循迹. */
          if (line_any_black()) { avoid_handover(1); break; }

          motor_set(AV_PASS_PWM, AV_PASS_PWM);
          if (avg_abs_travel() >= AV_PARALLEL_CNT
              || HAL_GetTick() - avoid_stage_tick > AV_STAGE_TIMEOUT_MS) {
            avoid_sub = AV_TURN_IN;
            avoid_stage_tick = HAL_GetTick();
            snapshot();
          }
          break;

        case AV_TURN_IN:
          /* ⑤ 转向黑线: 角度独立(AV_TURN_IN_DIFF), 比转出角缓.
           * 转向过程中传感器扫到黑线, 同样立即交给循迹. */
          if (line_any_black()) { avoid_handover(1); break; }

          motor_set(-avoid_side * AV_PIVOT_PWM, avoid_side * AV_PIVOT_PWM);
          if (turn_diff() >= AV_TURN_IN_DIFF
              || HAL_GetTick() - avoid_stage_tick > AV_STAGE_TIMEOUT_MS) {
            avoid_sub = AV_SEARCH;
            avoid_stage_tick = HAL_GetTick();
            search_retry_done = 0;   /* 新的找线阶段, 追加机会重新计数 */
          }
          break;

        case AV_SEARCH:
          /* ⑥ 斜着直行找线: 压到任意一路黑线就交给循迹.
           * 走 AV_SEARCH_RETRY_MS 还没看到线, 说明横移/角度估算有偏差,
           * 朝黑线再转 30° 继续找(只追加一次). */
          motor_set(AV_SEARCH_PWM, AV_SEARCH_PWM);

          if (line_any_black()) {
            avoid_handover(1);
          } else if (!search_retry_done
                     && HAL_GetTick() - avoid_stage_tick > AV_SEARCH_RETRY_MS) {
            search_retry_done = 1;
            avoid_sub = AV_SEARCH_TURN;
            avoid_stage_tick = HAL_GetTick();
            snapshot();
          } else if (HAL_GetTick() - avoid_stage_tick > AV_SEARCH_TIMEOUT_MS) {
            avoid_handover(0);
          }
          break;

        case AV_SEARCH_TURN:
          /* ⑥+ 追加转向: 朝黑线方向再转 30°, 然后回到直行找线. */
          motor_set(-avoid_side * AV_PIVOT_PWM, avoid_side * AV_PIVOT_PWM);

          if (line_any_black()) {
            avoid_handover(1);
          } else if (turn_diff() >= AV_SEARCH_RETRY_DIFF
                     || HAL_GetTick() - avoid_stage_tick > AV_STAGE_TIMEOUT_MS) {
            avoid_sub = AV_SEARCH;
            avoid_stage_tick = HAL_GetTick();
          }
          break;
      }
      break;
#endif

    case STATE_PUSH:
      /* 直接向前把障碍物顶开. 顶开后(前向读数不再极小)恢复循迹.
       * 规则允许撞开障碍物继续跑, 拿部分分数, 比卡在原地强. */
      motor_set(PUSH_PWM, PUSH_PWM);

      /* 退出条件(至少顶 PUSH_CLEAR_MS, 防止刚进来就退出):
       *   · 重新看到黑线   -> 优先恢复循迹(障碍顶不掉/已经让开都不再纠缠)
       *   · 前向不再贴死   -> 障碍已经被推开
       *   · 超过 PUSH_MAX_MS -> 兜底, 避免一直顶着 */
      if (((HAL_GetTick() - state_tick > PUSH_CLEAR_MS)
           && (line_any_black() || front_nearest() >= US_COLLISION_CM))
          || HAL_GetTick() - state_tick > PUSH_MAX_MS) {
        state = STATE_FOLLOW;
        if (line_any_black()) last_error = (int8_t)line_error();
        u_prev = 0;
        e_filt_q4 = (int16_t)(line_error() * 16);
        collide_arm = 0;
#if AVOID_ENABLE
        avoid_settle_until = HAL_GetTick() + AVOID_SETTLE_MS;
#endif
        finish_guard_until = HAL_GetTick() + FINISH_GUARD_MS;
      }
      break;

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
      printf("st=%d sub=%d side=%2d encL=%5ld encR=%5ld pwm=%4ld/%4ld line=%d%d%d%d err=%2d us=%d %d %d uLraw=%d\r\n",
             (int)state, (int)avoid_sub, (int)avoid_side,
             (long)encoder_left(), (long)encoder_right(),
             (long)motor_left_out(), (long)motor_right_out(),
             (lr >> 3) & 1, (lr >> 2) & 1, (lr >> 1) & 1, lr & 1,
             (int)line_error(),
             (int)ultrasonic_dist(US_CENTER), (int)ultrasonic_dist(US_LEFT),
             (int)ultrasonic_dist(US_RIGHT),
             (int)ultrasonic_last_raw(US_LEFT));
    }
  }
#endif
}
