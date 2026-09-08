#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "main.h"

#define US_CENTER 0   /* 前中 */
#define US_LEFT   1   /* 左前 */
#define US_RIGHT  2   /* 右前 */

void ultrasonic_init(void);
void ultrasonic_update(void);        /* 每调用一次测一路(轮流), 避免串扰 */
float ultrasonic_dist(uint8_t idx);  /* 距离(cm), 无回波返回 999 */

#endif
