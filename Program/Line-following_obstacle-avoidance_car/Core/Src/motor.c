#include "motor.h"
#include "board.h"

static int32_t out_l = 0, out_r = 0;   /* 记录实际输出的 PWM, 供调试打印 */

void motor_init(void) {
  HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_SET);  /* 使能 TB6612 */
}

int32_t motor_left_out(void) {
  return out_l;
}

int32_t motor_right_out(void) {
  return out_r;
}

void motor_set(int32_t left, int32_t right) {
  /* 左右轮补偿(百分比): 抵消重心偏移/电机个体差异.
     必须放在限幅之前, 否则补偿后的值可能越过 PWM_MAX. */
  left  = left  * MOTOR_L_GAIN / 100;
  right = right * MOTOR_R_GAIN / 100;

  /* 限幅 */
  if (left > PWM_MAX) left = PWM_MAX;
  else if (left < -PWM_MAX) left = -PWM_MAX;
  if (right > PWM_MAX) right = PWM_MAX;
  else if (right < -PWM_MAX) right = -PWM_MAX;

  out_l = left;
  out_r = right;

  /* 左轮 */
  if (left >= 0) {
    HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(PWM_TIM, PWM_CH_LEFT, (uint32_t)left);
  } else {
    HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(PWM_TIM, PWM_CH_LEFT, (uint32_t)(-left));
  }

  /* 右轮 */
  if (right >= 0) {
    HAL_GPIO_WritePin(MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(PWM_TIM, PWM_CH_RIGHT, (uint32_t)right);
  } else {
    HAL_GPIO_WritePin(MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(PWM_TIM, PWM_CH_RIGHT, (uint32_t)(-right));
  }
}

void motor_stop(void) {
  out_l = 0;
  out_r = 0;
  __HAL_TIM_SET_COMPARE(PWM_TIM, PWM_CH_LEFT, 0);
  __HAL_TIM_SET_COMPARE(PWM_TIM, PWM_CH_RIGHT, 0);
  HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN, GPIO_PIN_RESET);
}

void motor_brake(void) {
  out_l = 0;
  out_r = 0;
  __HAL_TIM_SET_COMPARE(PWM_TIM, PWM_CH_LEFT, 0);
  __HAL_TIM_SET_COMPARE(PWM_TIM, PWM_CH_RIGHT, 0);
  HAL_GPIO_WritePin(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN, GPIO_PIN_SET);
}
