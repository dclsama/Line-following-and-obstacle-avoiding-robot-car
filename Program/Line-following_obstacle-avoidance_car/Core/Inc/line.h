#ifndef __LINE_H
#define __LINE_H

#include "main.h"

void line_init(void);
void line_update(void);         /* 每个控制周期调用一次 */
int8_t line_error(void);        /* 循迹误差, 正=黑线偏右需右转 */
uint8_t line_all_black(void);   /* 四路全黑(起终点横线) */
uint8_t line_all_white(void);   /* 四路全白(丢线) */
uint8_t line_any_black(void);   /* 任一路黑(找线) */
uint8_t line_raw(void);         /* 4bit: [左远 左近 右近 右远], 1=黑 */

#endif
