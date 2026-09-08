#ifndef __BOARD_H
#define __BOARD_H

#include "main.h"
#include "tim.h"

/* ================= 引脚定义 ================= */

/* ---- 电机 (TB6612) ----
 * 若某轮方向反了, 交换该轮的 IN1/IN2 两个宏 */
#define MOTOR_L_IN1_PORT  GPIOB
#define MOTOR_L_IN1_PIN   GPIO_PIN_12     /* 左轮 AIN1 */
#define MOTOR_L_IN2_PORT  GPIOB
#define MOTOR_L_IN2_PIN   GPIO_PIN_13     /* 左轮 AIN2 */
#define MOTOR_R_IN1_PORT  GPIOB
#define MOTOR_R_IN1_PIN   GPIO_PIN_14     /* 右轮 BIN1 */
#define MOTOR_R_IN2_PORT  GPIOB
#define MOTOR_R_IN2_PIN   GPIO_PIN_15     /* 右轮 BIN2 */
#define STBY_PORT         GPIOB
#define STBY_PIN          GPIO_PIN_10     /* 高电平使能 */

/* ---- 循迹 4 路 (左远/左近/右近/右远) ----
 * 常见红外模块压黑线输出 0, 若你的模块相反改 LINE_BLACK_LEVEL 为 GPIO_PIN_SET */
#define LINE_BLACK_LEVEL  GPIO_PIN_RESET

#define LINE_L_PORT   GPIOB
#define LINE_L_PIN    GPIO_PIN_0    /* 左远 */
#define LINE_ML_PORT  GPIOB
#define LINE_ML_PIN   GPIO_PIN_1    /* 左近 */
#define LINE_MR_PORT  GPIOB
#define LINE_MR_PIN   GPIO_PIN_5    /* 右近 */
#define LINE_R_PORT   GPIOB
#define LINE_R_PIN    GPIO_PIN_6    /* 右远 */

/* ---- 超声波 TRIG/ECHO (前中/左前/右前), 按实际接线改 ---- */
#define US_C_TRIG_PORT  GPIOB
#define US_C_TRIG_PIN   GPIO_PIN_7
#define US_C_ECHO_PORT  GPIOA
#define US_C_ECHO_PIN   GPIO_PIN_11

#define US_L_TRIG_PORT  GPIOB
#define US_L_TRIG_PIN   GPIO_PIN_8
#define US_L_ECHO_PORT  GPIOA
#define US_L_ECHO_PIN   GPIO_PIN_12

#define US_R_TRIG_PORT  GPIOB
#define US_R_TRIG_PIN   GPIO_PIN_9
#define US_R_ECHO_PORT  GPIOB
#define US_R_ECHO_PIN   GPIO_PIN_3

/* ---- PWM ---- */
#define PWM_TIM         &htim1
#define PWM_CH_LEFT     TIM_CHANNEL_1   /* PA8 */
#define PWM_CH_RIGHT    TIM_CHANNEL_2   /* PA9 */
#define PWM_MAX         7199            /* 与 TIM1 Period 一致 */

/* ---- 编码器 ---- */
#define ENC_TIM_LEFT    &htim2
#define ENC_TIM_RIGHT   &htim3

/* ================= 可调参数 (现场标定) ================= */

/* 循迹误差权重: 中间近权重大小, 两侧远权重大 */
#define LINE_W_L    -2
#define LINE_W_ML   -1
#define LINE_W_MR   +1
#define LINE_W_R    +2

/* 循迹 PID */
#define BASE_PWM      2600    /* 基础前进速度(占空比 0~7199) */
#define LINE_KP       700     /* 比例系数 */
#define LINE_KD       400     /* 微分系数 */

/* 上电延时(ms), 给你放车、退开的时间 */
#define STARTUP_DELAY_MS  2000

/* 起终点: 全黑连续 N 次认定横线; 第 N 次全黑 = 终点停车 */
#define BLACK_DEBOUNCE     3
#define BLACK_EVENTS_STOP  2

/* 超声波 */
#define US_TIMEOUT_US      10000   /* 回波超时(10ms, 覆盖约1.7m) */
#define OBSTACLE_DIST_CM   18      /* 触发避障距离(cm) */
#define OBSTACLE_DEBOUNCE  2       /* 连续 N 次检测到才触发 */

/* 绕行 (编码器计数, 需实测标定! 方法见注释) */
#define TURN_PWM            2200    /* 原地转向占空比 */
#define STRAIGHT_PWM        2600    /* 绕行直行占空比 */
#define AVOID_TURN_CNT      2500    /* 约90°点转的单轮计数(标定) */
#define AVOID_STRAIGHT_CNT  4000    /* 绕过障碍的直行计数(标定) */

/* 找线 */
#define REACQUIRE_PWM       2000    /* 找线前进速度 */
#define REACQUIRE_STEER     600     /* 找线转弯偏置 */
#define REACQUIRE_MAX_CNT   6000    /* 找线最大计数(超则停, 防跑飞) */

/* 控制周期(ms) */
#define LOOP_MS             10

/* 调试打印(1开 0关), 打印到 USART2 */
#define DEBUG_PRINT         1

#endif
