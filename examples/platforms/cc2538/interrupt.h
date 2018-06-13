/*!
 * Nested Vector Interrupt Controller driver
 * (C) 2017 VRT Systems
 */

#ifndef _INTERRUPT_H
#define _INTERRUPT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

extern volatile uint32_t NVIC_TYPE;
extern volatile uint32_t NVIC_EN[5];
extern volatile uint32_t NVIC_DIS[5];
extern volatile uint32_t NVIC_PEND[5];
extern volatile uint32_t NVIC_UNPEND[5];
extern volatile uint32_t NVIC_ACTIVE[5];
extern volatile uint32_t NVIC_PRI[37];
extern volatile uint32_t NVIC_SWTRIG;

/*!
 * Determine the register index for a given interrupt channel.
 * This is valid for NVIC_EN, NVIC_DIS, NVIC_PEND, NVIC_UNPEND and NVIC_ACTIVE.
 */
#define NVIC_REG_IDX(ch)	((ch) >> 5)

/*!
 * Determine the bit number for a given interrupt channel.
 * This is valid for NVIC_EN, NVIC_DIS, NVIC_PEND, NVIC_UNPEND and NVIC_ACTIVE.
 */
#define NVIC_REG_BIT(ch)	((ch) & (0x1f))

/*!
 * Determine the bit mask for a given interrupt channel.
 * This is valid for NVIC_EN, NVIC_DIS, NVIC_PEND, NVIC_UNPEND and NVIC_ACTIVE.
 */
#define NVIC_REG_BITMASK(ch)	(1 << NVIC_REG_BIT(ch))

/*!
 * Enable the specified NVIC channel.
 */
static inline void nvic_enable(uint8_t ch) {
	if (ch > 147)
		return;

	NVIC_EN[NVIC_REG_IDX(ch)] = NVIC_REG_BITMASK(ch);
}

/*!
 * Disable the specified NVIC channel.
 */
static inline void nvic_disable(uint8_t ch) {
	if (ch > 147)
		return;

	NVIC_DIS[NVIC_REG_IDX(ch)] = NVIC_REG_BITMASK(ch);
}

/*!
 * Return true if the interrupt is enabled.
 */
static inline _Bool nvic_is_enabled(uint8_t ch) {
	if (ch > 147)
		return false;

	if (NVIC_EN[NVIC_REG_IDX(ch)] & NVIC_REG_BITMASK(ch))
		return true;

	return false;
}

/*!
 * Mark the specified NVIC channel as pending.
 */
static inline void nvic_pend(uint8_t ch) {
	if (ch > 147)
		return;

	NVIC_PEND[NVIC_REG_IDX(ch)] = NVIC_REG_BITMASK(ch);
}

/*!
 * Unmark the specified NVIC channel as pending.
 */
static inline void nvic_unpend(uint8_t ch) {
	if (ch > 147)
		return;

	NVIC_UNPEND[NVIC_REG_IDX(ch)] = NVIC_REG_BITMASK(ch);
}

/*!
 * Return true if the interrupt is pending.
 */
static inline _Bool nvic_is_pending(uint8_t ch) {
	if (ch > 147)
		return false;

	if (NVIC_PEND[NVIC_REG_IDX(ch)] & NVIC_REG_BITMASK(ch))
		return true;

	return false;
}

/*!
 * Return true if the interrupt is active.
 */
static inline _Bool nvic_is_active(uint8_t ch) {
	if (ch > 147)
		return false;

	if (NVIC_ACTIVE[NVIC_REG_IDX(ch)] & NVIC_REG_BITMASK(ch))
		return true;

	return false;
}

/*!
 * Get the priority of the given interrupt.
 */
static inline uint8_t nvic_get_priority(uint8_t ch) {
	if (ch > 147)
		return UINT8_MAX;

	/* Figure out index… there are 4 channels per register */
	uint8_t idx = ch >> 2;
	uint8_t bit = ((ch & 0x03) * 8) + 5;
	return NVIC_PRI[idx] >> bit;
}
/*!
 * Set the priority of the given interrupt.
 */
static inline void nvic_set_priority(uint8_t ch, uint8_t pri) {
	if (ch > 147)
		return;

	/* Figure out index… there are 4 channels per register */
	uint8_t idx = ch >> 2;
	uint8_t bit = ((ch & 0x03) * 8) + 5;

	uint32_t value = NVIC_PRI[idx];
	value &= ~(0xfful << bit);
	value |= ((uint32_t)(pri & 0x07)) << bit;
	NVIC_PRI[idx] = value;
}

#ifdef __cplusplus
}
#endif
#endif
