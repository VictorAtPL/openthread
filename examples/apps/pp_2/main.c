/*
 *  Copyright (c) 2016, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <openthread/config.h>
#include <openthread-core-config.h>
#include <assert.h>
#include <string.h>

#include <openthread/cli.h>
#include <openthread/diag.h>
#include <openthread/openthread.h>
// Piotr Podbielski
#include <openthread/link.h>
#include <openthread/ip6.h>
#include <openthread/thread.h>
#include <openthread/types.h>
#include <openthread/joiner.h>
// Piotr Podbielski
#include <openthread/platform/logging.h>

#include "platform.h"

void otTaskletsSignalPending(otInstance *aInstance)
{
    (void)aInstance;
}

//void OTCALL s_HandleJoinerCallback(otError aError, void *aContext)
//{
//	switch (aError)
//	{
//	case OT_ERROR_NONE:
//		//(void)aContext;
//	    otThreadSetEnabled(aContext, true);
//		break;
//	default:
//		(void)aContext;
//		break;
//	}
//}

int main(int argc, char *argv[])
{
	otInstance *sInstance;

pseudo_reset:
	PlatformInit(argc, argv);

	sInstance = otInstanceInitSingle();

	assert(sInstance);
	otCliUartInit(sInstance);

	// Set PanID
	otPanId panId = 0xABCD;
	otLinkSetPanId(sInstance, panId);

	// Set Channel
	otLinkSetChannel(sInstance, 11);

	// Set ExtPanID
	uint8_t aExtendedPanId[] = {0xAB, 0xCD, 0x11, 0x11, 0xAB, 0xCD, 0x22, 0x22};
	otThreadSetExtendedPanId(sInstance, aExtendedPanId);

	// Set ExtAddress
	otExtAddress aExtAddress;
	uint8_t aExtAddress_m8[] = {0x00, 0x12, 0x4B, 0x00, 0x06, 0x2C, 0xF8, 0x62};
	memcpy(aExtAddress.m8, aExtAddress_m8, 8);

	const otExtAddress *aExtAddressPtr = &aExtAddress;
	otLinkSetExtendedAddress(sInstance, aExtAddressPtr);

	// Set MasterKey
	otMasterKey aKey;
	uint8_t m8[] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
	memcpy(aKey.m8, m8, 16);
	const otMasterKey *aKeyPtr = &aKey;

	otThreadSetMasterKey(sInstance, aKeyPtr);

	// Enable ip6 stack
	otIp6SetEnabled(sInstance, true);

	int threadStarted = 0;

	while (!PlatformPseudoResetWasRequested())
	{
		otTaskletsProcess(sInstance);
		PlatformProcessDrivers(sInstance);

		if (threadStarted == 0) {
			if (otIp6IsEnabled(sInstance)) {
				otThreadSetEnabled(sInstance, true);
				threadStarted = 1;
			}
		}
	}

	otInstanceFinalize(sInstance);

	goto pseudo_reset;

	return 0;
}

/*
 * Provide, if required an "otPlatLog()" function
 */
#if OPENTHREAD_CONFIG_LOG_OUTPUT == OPENTHREAD_CONFIG_LOG_OUTPUT_APP
void otPlatLog(otLogLevel aLogLevel, otLogRegion aLogRegion, const char *aFormat, ...)
{
    OT_UNUSED_VARIABLE(aLogLevel);
    OT_UNUSED_VARIABLE(aLogRegion);
    OT_UNUSED_VARIABLE(aFormat);

    va_list ap;
    va_start(ap, aFormat);
    otCliPlatLogv(aLogLevel, aLogRegion, aFormat, ap);
    va_end(ap);
}
#endif
