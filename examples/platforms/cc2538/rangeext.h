/*!
 * TI CC2592 Range Extender Integration
 * (C) 2017 VRT Systems
 */

#ifndef _RANGEEXT_H
#define _RANGEEXT_H

#include <stdint.h>
#include <stdbool.h>

#include <openthread/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Configure the RF module to talk to the range extender.
 *
 * @param[in]	pa_en_pin	PA enable pin; or -1 for no-connect
 * @param[in]	lna_en_pin	LNA enable pin; or -1 for no-connect
 * @param[in]	hgm_port	HGM port number; or -1 for no-connect
 * @param[in]	hgm_pin		HGM pin number; or -1 for no-connect
 */
void rangeext_init(int8_t pa_en_pin, int8_t lna_en_pin,
		int8_t hgm_port, int8_t hgm_pin);

/*!
 * Force-Enable/Disable high-gain mode.
 *
 * @param[in]	force		Force the state of HGM… if false, then we
 * 				decide based on the average RSSI observed and
 * 				the thresholds set.
 * @param[in]	enable		Enable high-gain mode
 */
void rangeext_hgm(_Bool force, _Bool enable);

/*!
 * The low RSSI threshold.  If in automatic mode, and the average RSSI drops
 * below this set threshold, we switch on HGM to boost the weak signals.
 */
extern int8_t rangeext_low_threshold;

/*!
 * The high RSSI threshold.  If in automatic mode, and the average RSSI rises
 * above this set threshold, we switch off HGM as the signals ought to be
 * strong enough without.
 */
extern int8_t rangeext_high_threshold;

/*!
 * Check the RSSI status and adjust the HGM setting if needed.
 */
void rangeext_process(otInstance* instance);

/*!
 * Return the average RSSI last computed.
 */
int8_t rangeext_get_avg_rssi();

/*!
 * Return whether the HGM signal is asserted or not.
 */
_Bool rangeext_get_hgm();

#ifdef __cplusplus
}
#endif

#endif
