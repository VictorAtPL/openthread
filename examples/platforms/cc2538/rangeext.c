/*!
 * TI CC2592 Range Extender Integration
 * (C) 2017 VRT Systems
 */

#include "gpio.h"
#include "cc2538-reg.h"
#include "rangeext.h"

#include <openthread/thread.h>
#include <openthread/thread_ftd.h>

#if 0
#define RANGEEXT_TRACE
#endif

#ifdef RANGEEXT_TRACE
#include <stdio.h>
#endif

int8_t rangeext_low_threshold = -60;
int8_t rangeext_high_threshold = -20;
static _Bool rangeext_force_hgm = false;
static _Bool rangeext_hgm_state = false;
static int8_t rangeext_rssi = 0;

extern volatile uint32_t RFCORE_XREG_RFC_OBS_CTRL[3];
extern volatile uint32_t CCTEST_OBSSEL[8];

#define RFCORE_XREG_RFC_OBS_INV		(1 << 6)
#define RFCORE_XREG_RFC_OBS_SEL_PA_PD	050
#define RFCORE_XREG_RFC_OBS_SEL_LNA_PD	051

#define RFCORE_CCTEST_OBSSEL_EN		(1 << 7)

static uint32_t rangeext_hgm_bm = 0;	/*!< HGM GPIO bit mask */

static void rangeext_set_hgm(_Bool enable) {
#ifdef RANGEEXT_TRACE
	iprintf("%s:%d: hgm %s (was %s, GPIOs 0x%08lx)\r\n",
			__FILE__, __LINE__,
			(enable) ? "ON" : "OFF",
			(rangeext_hgm_state) ? "ON" : "OFF",
			rangeext_hgm_bm);
#endif
	if (rangeext_hgm_bm) {
		gpio_set_outputs(rangeext_hgm_bm, (enable) ? rangeext_hgm_bm : 0);
	}
	rangeext_hgm_state = enable;
}

/*!
 * Configure the RF module to talk to the range extender.
 *
 * @param[in]	pa_en_pin	PA enable pin; or -1 for no-connect
 * @param[in]	lna_en_pin	LNA enable pin; or -1 for no-connect
 * @param[in]	hgm_port	HGM port number; or -1 for no-connect
 * @param[in]	hgm_pin		HGM pin number; or -1 for no-connect
 */
void rangeext_init(int8_t pa_en_pin, int8_t lna_en_pin,
		int8_t hgm_port, int8_t hgm_pin) {
	uint32_t outputs = 0;

	if (pa_en_pin >= 0) {
		/* Make PCx an output */
		outputs |= 1 << GPIO_PC(pa_en_pin);

		/*
		 * Select observable output 0 to be PA power-down,
		 * and invert it to suit the CC2592.
		 */
		RFCORE_XREG_RFC_OBS_CTRL[0] = \
			(RFCORE_XREG_RFC_OBS_INV | \
			 RFCORE_XREG_RFC_OBS_SEL_PA_PD);

		/* Route observable 0 to PCx */
		CCTEST_OBSSEL[pa_en_pin] =  \
			(RFCORE_CCTEST_OBSSEL_EN | 0);
	}

	if (lna_en_pin >= 0) {
		outputs |= 1 << GPIO_PC(lna_en_pin);

		/*
		 * Select observable output 1 to be LNA power-down,
		 * and invert it to suit the CC2592.
		 */
		RFCORE_XREG_RFC_OBS_CTRL[1] = \
			(RFCORE_XREG_RFC_OBS_INV | \
			 RFCORE_XREG_RFC_OBS_SEL_LNA_PD);

		/* Route observable 1 to PCx */
		CCTEST_OBSSEL[lna_en_pin] =  \
			(RFCORE_CCTEST_OBSSEL_EN | 1);
	}

	if ((hgm_port >= 0) && (hgm_pin >= 0)) {
		uint8_t hgm_gpio = GPIO_NUM(hgm_port, hgm_pin);
		rangeext_hgm_bm = 1 << hgm_gpio;
		outputs |= rangeext_hgm_bm;
		GPIO_IOC_OVER[hgm_gpio] = GPIO_IOC_OVER_OUTPUT_EN;
	}

	/* Set up outputs */
	gpio_set_direction(1, outputs);

	/* Initialise state, default to HGM on until we know better */
	rangeext_hgm_state = false;
	rangeext_set_hgm(true);
}

/*
 * Enable/Disable high-gain mode.
 */
void rangeext_hgm(_Bool force, _Bool enable) {
	rangeext_force_hgm = force;
	if (force) {
		rangeext_set_hgm(enable);
	}
}

void rangeext_process(otInstance* instance) {
	if (rangeext_force_hgm) {
		/* We're being forced, don't do anything */
		return;
	}

	/*
	 * OpenThread doesn't give us an average as such, we get either the
	 * last value, or it averages on a per-node basis.  So hunt through
	 * the parents and children, and compute an overall average.
	 */
	int32_t rssi_sum = 0;
	uint32_t rssi_count = 0;
	{
		int8_t rssi;
		if (otThreadGetParentAverageRssi(instance, &rssi)
				== OT_ERROR_NONE) {
#ifdef RANGEEXT_TRACE
			iprintf("%s:%d: parent = %d\r\n",
					__FILE__, __LINE__, rssi);
#endif
			rssi_sum += rssi;
			rssi_count++;
		}
	}

	{
		uint8_t child_id = 0;
		otChildInfo info;
		while (otThreadGetChildInfoByIndex(instance, child_id, &info)
				== OT_ERROR_NONE) {
#ifdef RANGEEXT_TRACE
			iprintf("%s:%d: child = %d\r\n",
					__FILE__, __LINE__, info.mAverageRssi);
#endif
			rssi_sum += info.mAverageRssi;
			rssi_count++;
			child_id++;
		}
	}

	/* Compute average, or if no parents/children, assume poor rssi */
#ifdef RANGEEXT_TRACE
	iprintf("%s:%d: sum = %ld count = %lu\r\n",
			__FILE__, __LINE__, rssi_sum, rssi_count);
#endif
	if (rssi_count) {
		rssi_sum /= rssi_count;
	} else {
		rssi_sum = -INT8_MIN;
	}

	rangeext_rssi = rssi_sum;
#ifdef RANGEEXT_TRACE
	iprintf("%s:%d: rssi = %d, low = %d, high = %d, "
			"is_low = %s, is_high = %s\r\n",
			__FILE__, __LINE__,
			rangeext_rssi,
			rangeext_low_threshold,
			rangeext_high_threshold,
			(rangeext_rssi <= rangeext_low_threshold)
				? "YES" : "NO",
			(rangeext_rssi >= rangeext_high_threshold)
				? "YES" : "NO");
#endif

	/* Decide state of HGM */
	if (rangeext_rssi >= rangeext_high_threshold) {
		rangeext_set_hgm(false);
	} else if (rangeext_rssi <= rangeext_low_threshold) {
		rangeext_set_hgm(true);
	}
}

/*!
 * Return the average RSSI last computed.
 */
int8_t rangeext_get_avg_rssi() {
	return rangeext_rssi;
}

/*!
 * Return whether the HGM signal is asserted or not.
 */
_Bool rangeext_get_hgm() {
	return rangeext_hgm_state;
}
