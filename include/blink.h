#ifndef BLINK_H
#define BLINK_H

#include "task.h"
#include "uart.h"
#include <stdint.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

void taskBlink(uint32_t blink_delay);
void blink_setup(void);

#endif /* ifndef BLINK_H */