#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

void encoder_init(void);
void encoder_update(void);       /* 每个控制周期调用一次, 累计计数 */
int32_t encoder_left(void);      /* 左轮累计计数 */
int32_t encoder_right(void);     /* 右轮累计计数 */

#endif
