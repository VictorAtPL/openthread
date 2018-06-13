/*!
 * TI CC2538 GPIO Control interface
 * (C) 2017 VRT Systems
 */
#include "gpio.h"

static gpio_pin_handler gpio_handlers[32] = {
	NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
};

/*! Register interest in a pin */
void gpio_register(uint8_t gpio, gpio_pin_handler handler) {
	gpio_handlers[gpio] = handler;
}

static inline void gpio_int(uint8_t bits, uint8_t offset) {
	while (bits) {
		if ((bits & 1) && gpio_handlers[offset]) {
			(gpio_handlers[offset])(offset);
		}
		offset++;
		bits >>= 1;
	}
}

/* Handlers for GPIO interrupts */
void gpio_a_int(void) {
	gpio_int(GPIOA_MIS, GPIO_PA0);
}

void gpio_b_int(void) {
	gpio_int(GPIOB_MIS, GPIO_PB0);
}

void gpio_c_int(void) {
	gpio_int(GPIOC_MIS, GPIO_PC0);
}

void gpio_d_int(void) {
	gpio_int(GPIOD_MIS, GPIO_PD0);
}
