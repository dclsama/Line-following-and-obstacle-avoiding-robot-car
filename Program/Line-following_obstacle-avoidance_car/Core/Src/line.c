#include "line.h"
#include "board.h"

static uint8_t s1, s2, s3, s4;   /* 1=黑 */
static int8_t error;
static uint8_t all_black, all_white, any_black;

void line_init(void) {
}

void line_update(void) {
  s1 = (HAL_GPIO_ReadPin(LINE_L_PORT, LINE_L_PIN) == LINE_BLACK_LEVEL);
  s2 = (HAL_GPIO_ReadPin(LINE_ML_PORT, LINE_ML_PIN) == LINE_BLACK_LEVEL);
  s3 = (HAL_GPIO_ReadPin(LINE_MR_PORT, LINE_MR_PIN) == LINE_BLACK_LEVEL);
  s4 = (HAL_GPIO_ReadPin(LINE_R_PORT, LINE_R_PIN) == LINE_BLACK_LEVEL);

  error = LINE_W_L * s1 + LINE_W_ML * s2 + LINE_W_MR * s3 + LINE_W_R * s4;
  all_black = s1 && s2 && s3 && s4;
  all_white = !(s1 || s2 || s3 || s4);
  any_black = s1 || s2 || s3 || s4;
}

int8_t line_error(void) {
  return error;
}

uint8_t line_all_black(void) {
  return all_black;
}

uint8_t line_all_white(void) {
  return all_white;
}

uint8_t line_any_black(void) {
  return any_black;
}

uint8_t line_raw(void) {
  return (uint8_t)((s1 << 3) | (s2 << 2) | (s3 << 1) | s4);
}
