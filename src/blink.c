#include "FreeRTOS.h"
#include "blink.h"

void taskBlink(uint32_t blink_delay) {
    for (;;) {
        gpio_toggle(GPIOC, GPIO13);
		vTaskDelay(pdMS_TO_TICKS(blink_delay));
	}
}

void blink_setup(void) {
    // Enable clock for GPIO channel C
    rcc_periph_clock_enable(RCC_GPIOC);

    // Set pinmode for PC13
	gpio_set_mode(
		GPIOC,
		GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL,
		GPIO13);

	// Turn LED off
	gpio_set(GPIOC, GPIO13);
}