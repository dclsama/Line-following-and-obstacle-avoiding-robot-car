#include "utils.h"
#include "usart.h"

/* 用 DWT->CYCCNT 做微秒级计时, 72MHz 下每周期 1/72us */
void dwt_init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t micros(void) {
  return DWT->CYCCNT / (SystemCoreClock / 1000000UL);
}

void delay_us(uint32_t us) {
  uint32_t start = micros();
  while ((micros() - start) < us) {
  }
}

/* printf 重定向到 USART2 (syscalls.c 的 _write 会调用它) */
int __io_putchar(int ch) {
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
