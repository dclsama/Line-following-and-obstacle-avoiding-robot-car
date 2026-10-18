#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "main.h"

#define US_CENTER     0   /* 正前 */
#define US_LEFT       1   /* 左前 */
#define US_RIGHT      2   /* 右前 */
#define US_LEFT_SIDE  3   /* 左侧(垂直朝左) */
#define US_RIGHT_SIDE 4   /* 右侧(垂直朝右) */

void ultrasonic_init(void);
void ultrasonic_update(void);        /* 每调用一次测一路(轮流), 避免串扰 */
float ultrasonic_dist(uint8_t idx);  /* 中值滤波后的距离(cm), 无效返回 999 */

/* 最近一次【原始】读数(未经过有效性窗口和中值滤波), 仅供 DEBUG_PRINT 诊断用.
 * 想确认某个通道是不是一直在误报, 就对比它和 ultrasonic_dist() 的差异. */
float ultrasonic_last_raw(uint8_t idx);

#endif
