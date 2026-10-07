#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

void motor_init(void);
void motor_set(int32_t left, int32_t right);  /* -PWM_MAX ~ +PWM_MAX, 正=前进 */
void motor_stop(void);   /* 惰行(自由滑行) */
void motor_brake(void);  /* 短路制动 */
int32_t motor_left_out(void);   /* 当前左轮 PWM 输出(调试用) */
int32_t motor_right_out(void);  /* 当前右轮 PWM 输出(调试用) */

#endif
