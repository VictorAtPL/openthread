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
#include <openthread/platform/platform.h>
#include <openthread/platform/logging.h>

void otTaskletsSignalPending(otInstance *aInstance)
{
    (void)aInstance;
}

void OTCALL s_HandleJoinerCallback(otError aError, void *aContext)
{
	switch (aError)
	{
	case OT_ERROR_NONE:
		//(void)aContext;
	    otThreadSetEnabled(aContext, true);
		break;
	default:
		(void)aContext;
		break;
	}
}

int main(int argc, char *argv[])
{
	otInstance *sInstance;

	PlatformInit(argc, argv);

	sInstance = otInstanceInitSingle();

	assert(sInstance);
	otCliUartInit(sInstance);

	otIp6SetEnabled(sInstance, true);

	int joinerStarted = 0;

	while (1)
	{
		otTaskletsProcess(sInstance);
		PlatformProcessDrivers(sInstance);

		if (joinerStarted == 0) {
			if (otIp6IsEnabled(sInstance)) {
				otJoinerStart(sInstance, "WAT2018", NULL,
				  PACKAGE_NAME, OPENTHREAD_CONFIG_PLATFORM_INFO, PACKAGE_VERSION, NULL,
				  &s_HandleJoinerCallback, sInstance);
				joinerStarted = 1;
			}
		}
	}

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
