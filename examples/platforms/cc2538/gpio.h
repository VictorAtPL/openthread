/*!
 * TI CC2538 GPIO Control interface
 * (C) 2017 VRT Systems
 */
#ifndef _GPIO_H
#define _GPIO_H

#include "interrupt.h"

#include <stdlib.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Number of GPIO bits allocated.  The CC2538 has 8 pins per GPIO port, so
 * we'll allocate 3 bits.
 */
#define _GPIO_PIN_BITS			(3)

/*!
 * Helper macro: define a GPIO port number in terms of bit number.
 */
#define GPIO_PORT(port)			((port) << _GPIO_PIN_BITS)

#define GPIO_PORT_A			(0)
#define GPIO_PORT_B			(1)
#define GPIO_PORT_C			(2)
#define GPIO_PORT_D			(3)

/* Interrupt channels */
#define GPIO_IRQ_A			(0)
#define GPIO_IRQ_B			(1)
#define GPIO_IRQ_C			(2)
#define GPIO_IRQ_D			(3)

/*!
 * Helper macro: define a GPIO pin number in terms of bit number.
 */
#define GPIO_NUM(port, pin)		(GPIO_PORT(port) | (pin))

/*!
 * Helper macro: return the bit mask corresponding to a GPIO port/pin.
 */
#define GPIO_NUM_bm(port, pin)		(1 << GPIO_NUM(port, pin))

#define GPIO_PA(pin)			GPIO_NUM(GPIO_PORT_A, pin)
#define GPIO_PA0			GPIO_PA(0)
#define GPIO_PA1			GPIO_PA(1)
#define GPIO_PA2			GPIO_PA(2)
#define GPIO_PA3			GPIO_PA(3)
#define GPIO_PA4			GPIO_PA(4)
#define GPIO_PA5			GPIO_PA(5)
#define GPIO_PA6			GPIO_PA(6)
#define GPIO_PA7			GPIO_PA(7)

#define GPIO_PB(pin)			GPIO_NUM(GPIO_PORT_B, pin)
#define GPIO_PB0			GPIO_PB(0)
#define GPIO_PB1			GPIO_PB(1)
#define GPIO_PB2			GPIO_PB(2)
#define GPIO_PB3			GPIO_PB(3)
#define GPIO_PB4			GPIO_PB(4)
#define GPIO_PB5			GPIO_PB(5)
#define GPIO_PB6			GPIO_PB(6)
#define GPIO_PB7			GPIO_PB(7)

#define GPIO_PC(pin)			GPIO_NUM(GPIO_PORT_C, pin)
#define GPIO_PC0			GPIO_PC(0)
#define GPIO_PC1			GPIO_PC(1)
#define GPIO_PC2			GPIO_PC(2)
#define GPIO_PC3			GPIO_PC(3)
#define GPIO_PC4			GPIO_PC(4)
#define GPIO_PC5			GPIO_PC(5)
#define GPIO_PC6			GPIO_PC(6)
#define GPIO_PC7			GPIO_PC(7)

#define GPIO_PD(pin)			GPIO_NUM(GPIO_PORT_D, pin)
#define GPIO_PD0			GPIO_PD(0)
#define GPIO_PD1			GPIO_PD(1)
#define GPIO_PD2			GPIO_PD(2)
#define GPIO_PD3			GPIO_PD(3)
#define GPIO_PD4			GPIO_PD(4)
#define GPIO_PD5			GPIO_PD(5)
#define GPIO_PD6			GPIO_PD(6)
#define GPIO_PD7			GPIO_PD(7)

/* Bit masks for individual pins */

#define GPIO_PA0_bm			(1 << GPIO_PA0)
#define GPIO_PA1_bm			(1 << GPIO_PA1)
#define GPIO_PA2_bm			(1 << GPIO_PA2)
#define GPIO_PA3_bm			(1 << GPIO_PA3)
#define GPIO_PA4_bm			(1 << GPIO_PA4)
#define GPIO_PA5_bm			(1 << GPIO_PA5)
#define GPIO_PA6_bm			(1 << GPIO_PA6)
#define GPIO_PA7_bm			(1 << GPIO_PA7)

#define GPIO_PORT_A_bm	( GPIO_PA0_bm | GPIO_PA1_bm | GPIO_PA2_bm | GPIO_PA3_bm \
			| GPIO_PA4_bm | GPIO_PA5_bm | GPIO_PA6_bm | GPIO_PA7_bm )

#define GPIO_PB0_bm			(1 << GPIO_PB0)
#define GPIO_PB1_bm			(1 << GPIO_PB1)
#define GPIO_PB2_bm			(1 << GPIO_PB2)
#define GPIO_PB3_bm			(1 << GPIO_PB3)
#define GPIO_PB4_bm			(1 << GPIO_PB4)
#define GPIO_PB5_bm			(1 << GPIO_PB5)
#define GPIO_PB6_bm			(1 << GPIO_PB6)
#define GPIO_PB7_bm			(1 << GPIO_PB7)

#define GPIO_PORT_B_bm	( GPIO_PB0_bm | GPIO_PB1_bm | GPIO_PB2_bm | GPIO_PB3_bm \
			| GPIO_PB4_bm | GPIO_PB5_bm | GPIO_PB6_bm | GPIO_PB7_bm )

#define GPIO_PC0_bm			(1 << GPIO_PC0)
#define GPIO_PC1_bm			(1 << GPIO_PC1)
#define GPIO_PC2_bm			(1 << GPIO_PC2)
#define GPIO_PC3_bm			(1 << GPIO_PC3)
#define GPIO_PC4_bm			(1 << GPIO_PC4)
#define GPIO_PC5_bm			(1 << GPIO_PC5)
#define GPIO_PC6_bm			(1 << GPIO_PC6)
#define GPIO_PC7_bm			(1 << GPIO_PC7)

#define GPIO_PORT_C_bm	( GPIO_PC0_bm | GPIO_PC1_bm | GPIO_PC2_bm | GPIO_PC3_bm \
			| GPIO_PC4_bm | GPIO_PC5_bm | GPIO_PC6_bm | GPIO_PC7_bm )

#define GPIO_PD0_bm			(1 << GPIO_PD0)
#define GPIO_PD1_bm			(1 << GPIO_PD1)
#define GPIO_PD2_bm			(1 << GPIO_PD2)
#define GPIO_PD3_bm			(1 << GPIO_PD3)
#define GPIO_PD4_bm			(1 << GPIO_PD4)
#define GPIO_PD5_bm			(1 << GPIO_PD5)
#define GPIO_PD6_bm			(1 << GPIO_PD6)
#define GPIO_PD7_bm			(1 << GPIO_PD7)

#define GPIO_PORT_D_bm	( GPIO_PD0_bm | GPIO_PD1_bm | GPIO_PD2_bm | GPIO_PD3_bm \
			| GPIO_PD4_bm | GPIO_PD5_bm | GPIO_PD6_bm | GPIO_PD7_bm )

/* Port base addresses and registers */
extern volatile uint32_t GPIOA_BASE[16384];
extern volatile uint32_t GPIOA_DATA[256];
extern volatile uint32_t GPIOA_DIR;
extern volatile uint32_t GPIOA_IS;
extern volatile uint32_t GPIOA_IBE;
extern volatile uint32_t GPIOA_IEV;
extern volatile uint32_t GPIOA_IE;
extern volatile uint32_t GPIOA_RIS;
extern volatile uint32_t GPIOA_MIS;
extern volatile uint32_t GPIOA_IC;
extern volatile uint32_t GPIOA_AFSEL;
extern volatile uint32_t GPIOA_GPIOLOCK;
extern volatile uint32_t GPIOA_GPIOCR;
extern volatile uint32_t GPIOA_PMUX;
extern volatile uint32_t GPIOA_P_EDGE_CTRL;
extern volatile uint32_t GPIOA_PI_EN;
extern volatile uint32_t GPIOA_IRQ_DETECT_ACK;
extern volatile uint32_t GPIOA_USB_IRQ_ACK;
extern volatile uint32_t GPIOA_IRQ_DETECT_UNMASK;

extern volatile uint32_t GPIOB_BASE[16384];
extern volatile uint32_t GPIOB_DATA[256];
extern volatile uint32_t GPIOB_DIR;
extern volatile uint32_t GPIOB_IS;
extern volatile uint32_t GPIOB_IBE;
extern volatile uint32_t GPIOB_IEV;
extern volatile uint32_t GPIOB_IE;
extern volatile uint32_t GPIOB_RIS;
extern volatile uint32_t GPIOB_MIS;
extern volatile uint32_t GPIOB_IC;
extern volatile uint32_t GPIOB_AFSEL;
extern volatile uint32_t GPIOB_GPIOLOCK;
extern volatile uint32_t GPIOB_GPIOCR;
extern volatile uint32_t GPIOB_PMUX;
extern volatile uint32_t GPIOB_P_EDGE_CTRL;
extern volatile uint32_t GPIOB_PI_EN;
extern volatile uint32_t GPIOB_IRQ_DETECT_ACK;
extern volatile uint32_t GPIOB_USB_IRQ_ACK;
extern volatile uint32_t GPIOB_IRQ_DETECT_UNMASK;

extern volatile uint32_t GPIOC_BASE[16384];
extern volatile uint32_t GPIOC_DATA[256];
extern volatile uint32_t GPIOC_DIR;
extern volatile uint32_t GPIOC_IS;
extern volatile uint32_t GPIOC_IBE;
extern volatile uint32_t GPIOC_IEV;
extern volatile uint32_t GPIOC_IE;
extern volatile uint32_t GPIOC_RIS;
extern volatile uint32_t GPIOC_MIS;
extern volatile uint32_t GPIOC_IC;
extern volatile uint32_t GPIOC_AFSEL;
extern volatile uint32_t GPIOC_GPIOLOCK;
extern volatile uint32_t GPIOC_GPIOCR;
extern volatile uint32_t GPIOC_PMUX;
extern volatile uint32_t GPIOC_P_EDGE_CTRL;
extern volatile uint32_t GPIOC_PI_EN;
extern volatile uint32_t GPIOC_IRQ_DETECT_ACK;
extern volatile uint32_t GPIOC_USB_IRQ_ACK;
extern volatile uint32_t GPIOC_IRQ_DETECT_UNMASK;

extern volatile uint32_t GPIOD_BASE[16384];
extern volatile uint32_t GPIOD_DATA[256];
extern volatile uint32_t GPIOD_DIR;
extern volatile uint32_t GPIOD_IS;
extern volatile uint32_t GPIOD_IBE;
extern volatile uint32_t GPIOD_IEV;
extern volatile uint32_t GPIOD_IE;
extern volatile uint32_t GPIOD_RIS;
extern volatile uint32_t GPIOD_MIS;
extern volatile uint32_t GPIOD_IC;
extern volatile uint32_t GPIOD_AFSEL;
extern volatile uint32_t GPIOD_GPIOLOCK;
extern volatile uint32_t GPIOD_GPIOCR;
extern volatile uint32_t GPIOD_PMUX;
extern volatile uint32_t GPIOD_P_EDGE_CTRL;
extern volatile uint32_t GPIOD_PI_EN;
extern volatile uint32_t GPIOD_IRQ_DETECT_ACK;
extern volatile uint32_t GPIOD_USB_IRQ_ACK;
extern volatile uint32_t GPIOD_IRQ_DETECT_UNMASK;

/* Port register allocation address offsets; expressed as array indices */
#define GPIO_REG_DATA			(0x000 >> 2)
#define GPIO_REG_DIR			(0x400 >> 2)
#define GPIO_REG_IS			(0x404 >> 2)
#define GPIO_REG_IBE			(0x408 >> 2)
#define GPIO_REG_IEV			(0x40c >> 2)
#define GPIO_REG_IE			(0x410 >> 2)
#define GPIO_REG_RIS			(0x414 >> 2)
#define GPIO_REG_MIS			(0x418 >> 2)
#define GPIO_REG_IC			(0x41c >> 2)
#define GPIO_REG_AFSEL			(0x420 >> 2)
#define GPIO_REG_GPIOLOCK		(0x520 >> 2)
#define GPIO_REG_GPIOCR			(0x524 >> 2)
#define GPIO_REG_PMUX			(0x700 >> 2)
#define GPIO_REG_P_EDGE_CTRL		(0x704 >> 2)
#define GPIO_REG_PI_EN			(0x710 >> 2)
#define GPIO_REG_IRQ_DETECT_ACK		(0x718 >> 2)
#define GPIO_REG_USB_IRQ_ACK		(0x71c >> 2)
#define GPIO_REG_IRQ_DETECT_UNMASK	(0x720 >> 2)

#define GPIO_REG(port, reg)		(port[(reg) >> 2])
#define GPIO_NUM_PORT(gpio)		((gpio) >> _GPIO_PIN_BITS)

/* IOC I/O Mux selections */
#define GPIO_IOC_SEL_UART0_TXD		(0x00)
#define GPIO_IOC_SEL_UART1_RTS		(0x01)
#define GPIO_IOC_SEL_UART1_TXD		(0x02)
#define GPIO_IOC_SEL_SSI0_TXD		(0x03)
#define GPIO_IOC_SEL_SSI0_CLK		(0x04)
#define GPIO_IOC_SEL_SSI0_FSS		(0x05)
#define GPIO_IOC_SEL_SSI0_TXSER		(0x06)
#define GPIO_IOC_SEL_SSI1_TXD		(0x07)
#define GPIO_IOC_SEL_SSI1_CLK		(0x08)
#define GPIO_IOC_SEL_SSI1_FSS		(0x09)
#define GPIO_IOC_SEL_SSI1_TXSER		(0x0a)
#define GPIO_IOC_SEL_I2C_SDA		(0x0b)
#define GPIO_IOC_SEL_I2C_SCL		(0x0c)
#define GPIO_IOC_SEL_GPT0_CP1		(0x0d)
#define GPIO_IOC_SEL_GPT0_CP2		(0x0e)
#define GPIO_IOC_SEL_GPT1_CP1		(0x0f)
#define GPIO_IOC_SEL_GPT1_CP2		(0x10)
#define GPIO_IOC_SEL_GPT2_CP1		(0x11)
#define GPIO_IOC_SEL_GPT2_CP2		(0x12)
#define GPIO_IOC_SEL_GPT3_CP1		(0x13)
#define GPIO_IOC_SEL_GPT3_CP2		(0x14)

/* IOC I/O override selections */
#define GPIO_IOC_OVER_OUTPUT_EN		(1 << 3)
#define GPIO_IOC_OVER_PULLUP_EN		(1 << 2)
#define GPIO_IOC_OVER_PULLDOWN_EN	(1 << 1)
#define GPIO_IOC_OVER_ANALOGUE_EN	(1 << 0)

/* IOC output selection */
extern volatile uint32_t GPIO_IOC_SEL[32];
extern volatile uint32_t GPIO_IOC_SEL_PA;
extern volatile uint32_t GPIO_IOC_SEL_PA0;
extern volatile uint32_t GPIO_IOC_SEL_PA1;
extern volatile uint32_t GPIO_IOC_SEL_PA2;
extern volatile uint32_t GPIO_IOC_SEL_PA3;
extern volatile uint32_t GPIO_IOC_SEL_PA4;
extern volatile uint32_t GPIO_IOC_SEL_PA5;
extern volatile uint32_t GPIO_IOC_SEL_PA6;
extern volatile uint32_t GPIO_IOC_SEL_PA7;
extern volatile uint32_t GPIO_IOC_SEL_PB;
extern volatile uint32_t GPIO_IOC_SEL_PB0;
extern volatile uint32_t GPIO_IOC_SEL_PB1;
extern volatile uint32_t GPIO_IOC_SEL_PB2;
extern volatile uint32_t GPIO_IOC_SEL_PB3;
extern volatile uint32_t GPIO_IOC_SEL_PB4;
extern volatile uint32_t GPIO_IOC_SEL_PB5;
extern volatile uint32_t GPIO_IOC_SEL_PB6;
extern volatile uint32_t GPIO_IOC_SEL_PB7;
extern volatile uint32_t GPIO_IOC_SEL_PC;
extern volatile uint32_t GPIO_IOC_SEL_PC0;
extern volatile uint32_t GPIO_IOC_SEL_PC1;
extern volatile uint32_t GPIO_IOC_SEL_PC2;
extern volatile uint32_t GPIO_IOC_SEL_PC3;
extern volatile uint32_t GPIO_IOC_SEL_PC4;
extern volatile uint32_t GPIO_IOC_SEL_PC5;
extern volatile uint32_t GPIO_IOC_SEL_PC6;
extern volatile uint32_t GPIO_IOC_SEL_PC7;
extern volatile uint32_t GPIO_IOC_SEL_PD;
extern volatile uint32_t GPIO_IOC_SEL_PD0;
extern volatile uint32_t GPIO_IOC_SEL_PD1;
extern volatile uint32_t GPIO_IOC_SEL_PD2;
extern volatile uint32_t GPIO_IOC_SEL_PD3;
extern volatile uint32_t GPIO_IOC_SEL_PD4;
extern volatile uint32_t GPIO_IOC_SEL_PD5;
extern volatile uint32_t GPIO_IOC_SEL_PD6;
extern volatile uint32_t GPIO_IOC_SEL_PD7;

/* IOC override selection */
extern volatile uint32_t GPIO_IOC_OVER[32];
extern volatile uint32_t GPIO_IOC_OVER_PA;
extern volatile uint32_t GPIO_IOC_OVER_PA0;
extern volatile uint32_t GPIO_IOC_OVER_PA1;
extern volatile uint32_t GPIO_IOC_OVER_PA2;
extern volatile uint32_t GPIO_IOC_OVER_PA3;
extern volatile uint32_t GPIO_IOC_OVER_PA4;
extern volatile uint32_t GPIO_IOC_OVER_PA5;
extern volatile uint32_t GPIO_IOC_OVER_PA6;
extern volatile uint32_t GPIO_IOC_OVER_PA7;
extern volatile uint32_t GPIO_IOC_OVER_PB;
extern volatile uint32_t GPIO_IOC_OVER_PB0;
extern volatile uint32_t GPIO_IOC_OVER_PB1;
extern volatile uint32_t GPIO_IOC_OVER_PB2;
extern volatile uint32_t GPIO_IOC_OVER_PB3;
extern volatile uint32_t GPIO_IOC_OVER_PB4;
extern volatile uint32_t GPIO_IOC_OVER_PB5;
extern volatile uint32_t GPIO_IOC_OVER_PB6;
extern volatile uint32_t GPIO_IOC_OVER_PB7;
extern volatile uint32_t GPIO_IOC_OVER_PC;
extern volatile uint32_t GPIO_IOC_OVER_PC0;
extern volatile uint32_t GPIO_IOC_OVER_PC1;
extern volatile uint32_t GPIO_IOC_OVER_PC2;
extern volatile uint32_t GPIO_IOC_OVER_PC3;
extern volatile uint32_t GPIO_IOC_OVER_PC4;
extern volatile uint32_t GPIO_IOC_OVER_PC5;
extern volatile uint32_t GPIO_IOC_OVER_PC6;
extern volatile uint32_t GPIO_IOC_OVER_PC7;
extern volatile uint32_t GPIO_IOC_OVER_PD;
extern volatile uint32_t GPIO_IOC_OVER_PD0;
extern volatile uint32_t GPIO_IOC_OVER_PD1;
extern volatile uint32_t GPIO_IOC_OVER_PD2;
extern volatile uint32_t GPIO_IOC_OVER_PD3;
extern volatile uint32_t GPIO_IOC_OVER_PD4;
extern volatile uint32_t GPIO_IOC_OVER_PD5;
extern volatile uint32_t GPIO_IOC_OVER_PD6;
extern volatile uint32_t GPIO_IOC_OVER_PD7;

/* IOC UART0 control lines */
extern volatile uint32_t GPIO_IOC_UART0_RXD;

/* IOC UART1 control lines */
extern volatile uint32_t GPIO_IOC_UART1_CTS;
extern volatile uint32_t GPIO_IOC_UART1_RXD;

/* IOC SSI0 control lines */
extern volatile uint32_t GPIO_IOC_SSI0_CLK;
extern volatile uint32_t GPIO_IOC_SSI0_RXD;
extern volatile uint32_t GPIO_IOC_SSI0_FSS;
extern volatile uint32_t GPIO_IOC_SSI0_CLK_SSIIN;

/* IOC SSI1 control lines */
extern volatile uint32_t GPIO_IOC_SSI1_CLK;
extern volatile uint32_t GPIO_IOC_SSI1_RXD;
extern volatile uint32_t GPIO_IOC_SSI1_FSS;
extern volatile uint32_t GPIO_IOC_SSI1_CLK_SSIIN;

/* IOC I²C master control lines */
extern volatile uint32_t GPIO_IOC_I2CM_SDA;
extern volatile uint32_t GPIO_IOC_I2CM_SCL;

/* IOC General Purpose timer control lines */
extern volatile uint32_t GPIO_IOC_GPT0_OCP1;
extern volatile uint32_t GPIO_IOC_GPT0_OCP2;
extern volatile uint32_t GPIO_IOC_GPT1_OCP1;
extern volatile uint32_t GPIO_IOC_GPT1_OCP2;
extern volatile uint32_t GPIO_IOC_GPT2_OCP1;
extern volatile uint32_t GPIO_IOC_GPT2_OCP2;
extern volatile uint32_t GPIO_IOC_GPT3_OCP1;
extern volatile uint32_t GPIO_IOC_GPT3_OCP2;

/*!
 * Retrieve the base address of the GPIO register given a port number
 */
static inline volatile uint32_t* gpio_addr_from_port(uint8_t portnum) {
	switch (portnum) {
	case GPIO_PORT_A:
		return GPIOA_BASE;
	case GPIO_PORT_B:
		return GPIOB_BASE;
	case GPIO_PORT_C:
		return GPIOC_BASE;
	case GPIO_PORT_D:
		return GPIOD_BASE;
	default:
		return NULL;
	}
}

/*!
 * Set the given register according to the bits given in the mask.
 * This is a convenience for managing most registers on the four GPIO ports in
 * one hit.
 *
 * @param[in]	reg	Register offset
 * @param[in]	mask	Bit field of affected GPIOs
 * @param[in]	value	Values to set in the GPIO registers
 */
static inline void gpio_set_reg(uint16_t reg, uint32_t mask, uint32_t value) {
	if (mask & GPIO_PORT_A_bm) {
		uint32_t regmask = (mask >> GPIO_PORT(0)) & 0x000000fful;
		uint32_t regval = GPIOA_BASE[reg] & ~regmask;
		regval |= (value >> GPIO_PORT(0)) & regmask;
		GPIOA_BASE[reg] = regval;
	}

	if (mask & GPIO_PORT_B_bm) {
		uint32_t regmask = (mask >> GPIO_PORT(1)) & 0x000000fful;
		uint32_t regval = GPIOB_BASE[reg] & ~regmask;
		regval |= (value >> GPIO_PORT(1)) & regmask;
		GPIOB_BASE[reg] = regval;
	}

	if (mask & GPIO_PORT_C_bm) {
		uint32_t regmask = (mask >> GPIO_PORT(2)) & 0x000000fful;
		uint32_t regval = GPIOC_BASE[reg] & ~regmask;
		regval |= (value >> GPIO_PORT(2)) & regmask;
		GPIOC_BASE[reg] = regval;
	}

	if (mask & GPIO_PORT_D_bm) {
		uint32_t regmask = (mask >> GPIO_PORT(3)) & 0x000000fful;
		uint32_t regval = GPIOD_BASE[reg] & ~regmask;
		regval |= (value >> GPIO_PORT(3)) & regmask;
		GPIOD_BASE[reg] = regval;
	}
}

/*!
 * Get the value of the given register according to the bits given in the mask.
 * This is a convenience for managing most registers on the four GPIO ports in
 * one hit.
 *
 * @param[in]	reg	Register offset
 * @param[in]	mask	Bit field of affected GPIOs
 */
static inline uint32_t gpio_get_reg(uint16_t reg, uint32_t mask) {
	uint32_t value = 0;
	if (mask & GPIO_PORT_A_bm) {
		uint32_t regmask = (mask >> GPIO_PORT(0)) & 0x000000fful;
		uint32_t regval = GPIOA_BASE[reg] & regmask;
		value |= regval << GPIO_PORT(0);
	}

	if (mask & GPIO_PORT_B_bm) {
		uint32_t regmask = (mask >> GPIO_PORT(1)) & 0x000000fful;
		uint32_t regval = GPIOB_BASE[reg] & regmask;
		value |= regval << GPIO_PORT(1);
	}

	if (mask & GPIO_PORT_C_bm) {
		uint32_t regmask = (mask >> GPIO_PORT(2)) & 0x000000fful;
		uint32_t regval = GPIOC_BASE[reg] & regmask;
		value |= regval << GPIO_PORT(2);
	}

	if (mask & GPIO_PORT_D_bm) {
		uint32_t regmask = (mask >> GPIO_PORT(3)) & 0x000000fful;
		uint32_t regval = GPIOD_BASE[reg] & regmask;
		value |= regval << GPIO_PORT(3);
	}

	return value;
}

/*!
 * Make the given pins in the bit mask inputs.
 *
 * @param[in]	dir	Direction: 1=output, 0=input
 * @param[in]	mask	GPIOs affected as a bit field
 */
static inline void gpio_set_direction(uint8_t dir, uint32_t mask) {
	gpio_set_reg(GPIO_REG_DIR, mask, (dir) ? mask : 0);
}

/* Interrupt mode bits */
#define GPIO_INT_MODEBIT_IS		(0)
#define GPIO_INT_MODEBIT_IBE		(1)
#define GPIO_INT_MODEBIT_IEV		(2)
#define GPIO_INT_MODEBIT_IE		(3)

/* Interrupt modes */

/*! Helper macro for constructing mode settings */
#define _GPIO_INT_MODE(is, ibe, iev, ie)	\
	(	((is) << GPIO_INT_MODEBIT_IS)	\
	|	((ibe) << GPIO_INT_MODEBIT_IBE)	\
	|	((iev) << GPIO_INT_MODEBIT_IEV)	\
	|	((ie) << GPIO_INT_MODEBIT_IE))

/*! Pin is not enabled for interrupt handling */
#define GPIO_INT_MODE_DISABLE		_GPIO_INT_MODE(0, 0, 0, 0)
/*! Pin triggers interrupt on falling edge */
#define GPIO_INT_MODE_FALLING		_GPIO_INT_MODE(0, 0, 0, 1)
/*! Pin triggers interrupt on logic low */
#define GPIO_INT_MODE_LOW		_GPIO_INT_MODE(1, 0, 0, 1)
/*! Pin triggers interrupt on rising edge */
#define GPIO_INT_MODE_RISING		_GPIO_INT_MODE(0, 0, 1, 1)
/*! Pin triggers interrupt on logic high */
#define GPIO_INT_MODE_HIGH		_GPIO_INT_MODE(1, 0, 1, 1)
/*! Pin triggers interrupt on both edges */
#define GPIO_INT_MODE_BOTHEDGES		_GPIO_INT_MODE(0, 1, 0, 1)

/*!
 * Configure the interrupt configuration for the selected bits.
 *
 * @param[in]	mode	The interrupt mode to set.
 * @param[in]	mask	GPIOs affected as a bit field.
 */
static inline void gpio_set_int(uint8_t mode, uint32_t mask) {
	gpio_set_reg(GPIO_REG_IS, mask, (mode & (1 << GPIO_INT_MODEBIT_IS)) ? mask : 0);
	gpio_set_reg(GPIO_REG_IBE, mask, (mode & (1 << GPIO_INT_MODEBIT_IBE)) ? mask : 0);
	gpio_set_reg(GPIO_REG_IEV, mask, (mode & (1 << GPIO_INT_MODEBIT_IEV)) ? mask : 0);
	gpio_set_reg(GPIO_REG_IE, mask, (mode & (1 << GPIO_INT_MODEBIT_IE)) ? mask : 0);

	/* NVIC handling */
	uint32_t ie = gpio_get_reg(GPIO_REG_IE, mask);
	if (mask & GPIO_PORT_A_bm) {
		if (ie & GPIO_PORT_A_bm)
			nvic_enable(GPIO_IRQ_A);
		else
			nvic_disable(GPIO_IRQ_A);
	}
	if (mask & GPIO_PORT_B_bm) {
		if (ie & GPIO_PORT_B_bm)
			nvic_enable(GPIO_IRQ_B);
		else
			nvic_disable(GPIO_IRQ_B);
	}
	if (mask & GPIO_PORT_C_bm) {
		if (ie & GPIO_PORT_C_bm)
			nvic_enable(GPIO_IRQ_C);
		else
			nvic_disable(GPIO_IRQ_C);
	}
	if (mask & GPIO_PORT_D_bm) {
		if (ie & GPIO_PORT_D_bm)
			nvic_enable(GPIO_IRQ_D);
		else
			nvic_disable(GPIO_IRQ_D);
	}
}

/*! Set the I/O state of the given pins high or low */
static inline void gpio_set_outputs(uint32_t mask, uint32_t state) {
	if (mask & GPIO_PORT_A_bm) {
		uint32_t regmask = (mask >> GPIO_PORT(0)) & 0x000000fful;
		GPIOA_DATA[regmask] = state >> GPIO_PORT(0);
	}

	if (mask & GPIO_PORT_B_bm) {
		uint32_t regmask = (mask >> GPIO_PORT(1)) & 0x000000fful;
		GPIOB_DATA[regmask] = state >> GPIO_PORT(1);
	}

	if (mask & GPIO_PORT_C_bm) {
		uint32_t regmask = (mask >> GPIO_PORT(2)) & 0x000000fful;
		GPIOC_DATA[regmask] = state >> GPIO_PORT(2);
	}

	if (mask & GPIO_PORT_D_bm) {
		uint32_t regmask = (mask >> GPIO_PORT(3)) & 0x000000fful;
		GPIOD_DATA[regmask] = state >> GPIO_PORT(3);
	}
}

/*! Read the I/O state of the given pins */
static inline uint32_t gpio_get_inputs(uint32_t mask) {
	uint32_t state = 0;
	if (mask & GPIO_PORT_A_bm) {
		uint32_t regmask = (mask >> GPIO_PORT(0)) & 0x000000fful;
		state |= GPIOA_DATA[regmask] << GPIO_PORT(0);
	}

	if (mask & GPIO_PORT_B_bm) {
		uint32_t regmask = (mask >> GPIO_PORT(1)) & 0x000000fful;
		state |= GPIOB_DATA[regmask] << GPIO_PORT(1);
	}

	if (mask & GPIO_PORT_C_bm) {
		uint32_t regmask = (mask >> GPIO_PORT(2)) & 0x000000fful;
		state |= GPIOC_DATA[regmask] << GPIO_PORT(2);
	}

	if (mask & GPIO_PORT_D_bm) {
		uint32_t regmask = (mask >> GPIO_PORT(3)) & 0x000000fful;
		state |= GPIOD_DATA[regmask] << GPIO_PORT(3);
	}

	return state;
}

/*! Read the raw interrupt states for all registers */
static inline uint32_t gpio_get_raw_int(uint32_t mask) {
	return gpio_get_reg(GPIO_REG_RIS, mask);
}

/*! Read the masked interrupt states for all registers */
static inline uint32_t gpio_get_masked_int(uint32_t mask) {
	return gpio_get_reg(GPIO_REG_MIS, mask);
}

/*! Clear the interrupt states for all registers */
static inline void gpio_clear_int(uint32_t mask) {
	gpio_set_reg(GPIO_REG_IC, mask, mask);
}

/*!
 * Select an alternate function on the given pin.
 *
 * @param[in]	af	AFSEL setting: 1 = alternate function, 0 = GPIO
 * @param[in]	mask	Bit mask
 */
static inline void gpio_set_af(uint8_t af, uint32_t mask) {
	gpio_set_reg(GPIO_REG_AFSEL, mask, (af) ? mask : 0);
}

/*! Special unlock value for GPIOs ("LOCK") */
#define GPIO_UNLOCK_MAGIC	(0x4c4f434b)

/*! Lock the GPIOs */
static inline void gpio_lock(uint32_t mask) {
	if (mask & GPIO_PORT_A_bm)
		GPIOA_GPIOLOCK = 0;

	if (mask & GPIO_PORT_B_bm)
		GPIOB_GPIOLOCK = 0;

	if (mask & GPIO_PORT_C_bm)
		GPIOC_GPIOLOCK = 0;

	if (mask & GPIO_PORT_D_bm)
		GPIOD_GPIOLOCK = 0;
}

/*! Unlock the GPIOs */
static inline void gpio_unlock(uint32_t mask) {
	if (mask & GPIO_PORT_A_bm)
		GPIOA_GPIOLOCK = GPIO_UNLOCK_MAGIC;

	if (mask & GPIO_PORT_B_bm)
		GPIOB_GPIOLOCK = GPIO_UNLOCK_MAGIC;

	if (mask & GPIO_PORT_C_bm)
		GPIOC_GPIOLOCK = GPIO_UNLOCK_MAGIC;

	if (mask & GPIO_PORT_D_bm)
		GPIOD_GPIOLOCK = GPIO_UNLOCK_MAGIC;
}

/*! Commit the AFSEL selections for the given GPIOs */
static inline void gpio_commit_af(uint32_t mask) {
	gpio_set_reg(GPIO_REG_GPIOCR, mask, mask);
}

/*! Handler for pin changes */
typedef void (* gpio_pin_handler)(uint8_t gpio);

/*! Register interest in a pin */
void gpio_register(uint8_t gpio, gpio_pin_handler handler);

/* Handlers for GPIO interrupts */
void gpio_a_int(void);
void gpio_b_int(void);
void gpio_c_int(void);
void gpio_d_int(void);

#ifdef __cplusplus
}
#endif

#endif
