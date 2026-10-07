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
/* 右轮: 实测转向与左轮相反, 故 IN1/IN2 引脚对调使两轮方向一致 */
#define MOTOR_R_IN1_PORT  GPIOB
#define MOTOR_R_IN1_PIN   GPIO_PIN_15     /* 右轮 BIN2(已对调) */
#define MOTOR_R_IN2_PORT  GPIOB
#define MOTOR_R_IN2_PIN   GPIO_PIN_14     /* 右轮 BIN1(已对调) */
#define STBY_PORT         GPIOB
#define STBY_PIN          GPIO_PIN_10     /* 高电平使能 */

/* ---- 循迹 4 路 (左远/左近/右近/右远) ----
 * 黑线对应的电平: 压黑线时读出该电平即认为"检测到黑线".
 * 实测本模块: 黑线->低电平(指示灯亮), 白底->高电平(灯灭), 故为 RESET */
#define LINE_BLACK_LEVEL  GPIO_PIN_RESET

/* 避障功能总开关: 0=只调循迹(避障不触发), 1=开启避障(需先验证超声波读数正常) */
#define AVOID_ENABLE      1

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

/* ---- 超声波 侧面 (左/右, 垂直朝外) ---- */
#define US_LS_TRIG_PORT  GPIOB
#define US_LS_TRIG_PIN   GPIO_PIN_4
#define US_LS_ECHO_PORT  GPIOA
#define US_LS_ECHO_PIN   GPIO_PIN_10

#define US_RS_TRIG_PORT  GPIOB
#define US_RS_TRIG_PIN   GPIO_PIN_11
#define US_RS_ECHO_PORT  GPIOA
#define US_RS_ECHO_PIN   GPIO_PIN_15

/* ---- PWM ---- */
#define PWM_TIM         &htim1
#define PWM_CH_LEFT     TIM_CHANNEL_1   /* PA8 */
#define PWM_CH_RIGHT    TIM_CHANNEL_2   /* PA9 */
#define PWM_MAX         7199            /* 与 TIM1 Period 一致 */

/* ---- 编码器 ---- */
#define ENC_TIM_LEFT    &htim2
#define ENC_TIM_RIGHT   &htim3

/* ================= 可调参数 (现场标定) ================= */

/* 板载 LED (PC13, 低电平点亮) —— 无串口时用来观察小车状态 */
#define LED_PORT   GPIOC
#define LED_PIN    GPIO_PIN_13
#define LED_ON()   HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET)
#define LED_OFF()  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET)

/* 循迹误差权重: 中间近权重大小, 两侧远权重大 */
#define LINE_W_L    -2
#define LINE_W_ML   -1
#define LINE_W_MR   +1
#define LINE_W_R    +2

/* 循迹 PID —— 采用"非线性增益": 小误差用小增益(直线稳), 大误差用大增益(急弯猛),
 * 这样单靠一组参数无法兼顾的"直线不震荡"与"急弯跟得住"就同时满足了.
 * 注: 整体车速已按原值的 2/3 缩放 —— 转向量(增益)必须跟着一起缩,
 *     否则只降基础速度会让转向相对车速变得更猛, 手感就变了. */
#define BASE_PWM       1130   /* 基础前进速度(原1700的2/3) */
#define LINE_KP         330   /* 小误差(|e|<=1)增益(原500) */
#define LINE_KP_BIG     930   /* 大误差(|e|>=2)增益(原1400) */
#define LINE_KD         270   /* 微分系数(原400) */
#define LINE_LOST_U    1600   /* 丢线时的转向量(原2400): 朝上次偏差方向急转找线 */

/* 上电延时(ms), 给你放车、退开的时间 */
#define STARTUP_DELAY_MS  2000

/* 终点检测: 全黑连续 N 次认定横线(去抖). 上电即开始循迹, 第一次全黑即终点 */
#define BLACK_DEBOUNCE     3

/* 编码器计数 -> 厘米 换算 (必须实测标定!)
 * 标定法: 让车直行 50cm, 读串口 encL 前后差值 delta, 本值 = delta / 50 */
#define ENCODER_CNT_PER_CM   100

/* 越线停车: 车身长 12cm, 检测到终点线后继续前进此距离再停, 让整车越过终点线 */
#define FINISH_EXTRA_CM      12
#define FINISH_EXTRA_CNT     (FINISH_EXTRA_CM * ENCODER_CNT_PER_CM)
#define FINISH_PWM           1000    /* 越线前进速度(原1500的2/3, 慢速直行便于停准) */

/* 超声波 */
#define US_TIMEOUT_US         10000   /* 回波超时(10ms, 覆盖约1.7m) */
#define US_MAX_NEAR_JUMP      15      /* 单次"突然变近"超过此值视为干扰, 限幅(cm) */
/* 前方/斜前触发绕障的距离(cm).
 * 注意 HC-SR04 有约 2cm 盲区: 障碍贴到传感器上反而测不到(输出无回波),
 * 所以阈值不能贴着盲区设, 要留出余量, 让车在障碍逼近盲区之前就触发.
 * 代价是设太大容易把路过的远处障碍也当成需要绕行的目标. */
#define OBSTACLE_DIST_CM      15

/* 避障(状态机绕行). 赛道障碍基本出现在【侧前方】, 据此组织判断:
 *   触发 : 左前/右前的斜装传感器(不是侧向的)
 *   转回 : 【侧向】传感器确认旁边安全了 -> 可以绕回巡线
 *   全程降速
 * 用侧向传感器判断"能不能绕回"是有道理的: 车一旦开始转向, 斜前传感器的
 * 朝向也跟着转, 它的"距离变大"并不代表真的让开了; 而侧向传感器测的是垂直
 * 方向的净空, 更接近"横向有没有东西挡着". */
#define OBSTACLE_DEBOUNCE     2       /* 连续N次确认才触发绕行(去抖) */
#define SIDE_SAFE_DIST_CM     20      /* 障碍侧"侧向"超过此值 => 可安全绕回巡线(cm) */

/* 绕行速度. 注意: 转弯半径 R = (轮距/2)×(vL+vR)/(vR-vL),
 * 分子是"和"(基础速度), 分母是"差"(差速). 想只提速不改半径, 两者必须同比例调. */
#define AV_TURN_PWM           1050    /* 绕行转向基础速度: 提高后内轮脱离电机死区 */
#define AV_TURN_DIFF          310     /* 绕行转向差速: 与基础速度同比例放大, 保持半径不变 */
/* 转向方向. 取 +1 时: 障碍在右(avoid_side=-1) -> 左轮变慢/右轮变快 -> 车向左转,
 * 即远离障碍; 障碍在左时反之. 若实测车朝障碍那一侧转, 改成 (-1). */
#define AV_TURN_SIGN          (+1)
/* 让开多少 = R×(1-cosθ), 只取决于【转角θ】和半径R, 与车速无关.
 * 想绕得更开就加大本值(转角), 而不是提车速. */
#define AV_TURN_MAX_DIFF      1000    /* 让开阶段转角上限 => 偏角约50° (900≈45°, 每度约20计数) */

#define AV_PASS_PWM           950     /* 绕过阶段前进速度: 同样为提高内轮PWM而抬升 */
#define AV_PASS_DIFF          110     /* 绕过阶段转向差速: 与基础速度同比例放大, 保持半径不变 */
#define AV_PASS_MIN_CNT       1500    /* 绕过阶段最短距离: 确保真的从障碍旁经过 */
#define AV_PASS_MAX_CNT       5000    /* 绕过阶段兜底距离 */
#define AV_PASS_MAX_DIFF      600     /* 绕过阶段累计转角上限(约30°): 弧线会持续累积偏角,
                                         必须设上限, 否则车头会越绕越横 */

/* 转回阶段 —— 独立参数, 不再和"让开阶段"共用. 转回需要的是【平缓】:
 * 转向差速小 => 半径大 => 不会像原地打转. */
#define AV_BACK_PWM           950     /* 转回阶段前进速度 */
#define AV_BACK_DIFF          90      /* 转回阶段转向差速: 小 => 半径约5.3倍轮距, 转得平缓 */
#define AV_BACK_MAX_DIFF      1200    /* 转回阶段偏角上限; 超了强制交还循迹(防转不停) */

/* 控制周期(ms) */
#define LOOP_MS             10

/* 调试打印(1开 0关), 打印到 USART2 */
#define DEBUG_PRINT         1

#endif
