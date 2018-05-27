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

#include <openthread/diag.h>
#include <openthread/ncp.h>
//#include <openthread/cli.h>
#include <openthread/openthread.h>
// Piotr Podbielski
#include <openthread/link.h>
#include <openthread/ip6.h>
#include <openthread/thread.h>
#include <openthread/types.h>
#include <openthread/commissioner.h>
// Piotr Podbielski
#include <openthread/platform/logging.h>

#include "platform.h"
#include "cc2538-reg.h"

void otTaskletsSignalPending(otInstance *aInstance)
{
    (void)aInstance;
}

//int Hex2Bin(const char *aHex, uint8_t *aBin, uint16_t aBinLength)
//{
//    size_t hexLength = strlen(aHex);
//    const char *hexEnd = aHex + hexLength;
//    uint8_t *cur = aBin;
//    uint8_t numChars = hexLength & 1;
//    uint8_t byte = 0;
//
//    if ((hexLength + 1) / 2 > aBinLength)
//    {
//        return -1;
//    }
//
//    while (aHex < hexEnd)
//    {
//        if ('A' <= *aHex && *aHex <= 'F')
//        {
//            byte |= 10 + (*aHex - 'A');
//        }
//        else if ('a' <= *aHex && *aHex <= 'f')
//        {
//            byte |= 10 + (*aHex - 'a');
//        }
//        else if ('0' <= *aHex && *aHex <= '9')
//        {
//            byte |= *aHex - '0';
//        }
//        else
//        {
//            return -1;
//        }
//
//        aHex++;
//        numChars++;
//
//        if (numChars >= 2)
//        {
//            numChars = 0;
//            *cur++ = byte;
//            byte = 0;
//        }
//        else
//        {
//            byte <<= 4;
//        }
//    }
//
//    return (int)(cur - aBin);
//}

int main(int argc, char *argv[])
{
    otInstance *sInstance;

pseudo_reset:
    PlatformInit(argc, argv);

    sInstance = otInstanceInitSingle();

    assert(sInstance);
//    otCliUartInit(sInstance);

    otNcpInit(sInstance);

    // Set ExtAddress
	otExtAddress aExtAddress;
	uint8_t aExtAddress_m8[] = {0x00, 0x12, 0x4B, 0x00, 0x06, 0x2C, 0xF8, 0x4D};
	memcpy(aExtAddress.m8, aExtAddress_m8, 8);

	const otExtAddress *aExtAddressPtr = &aExtAddress;
	otLinkSetExtendedAddress(sInstance, aExtAddressPtr);

//    otPanId panId = 0x1234;
//    otLinkSetPanId(sInstance, panId);
//
//    otLinkSetChannel(sInstance, 11);
//
//    otIp6SetEnabled(sInstance, true);
//
//    int threadStarted = 0;
//    int commissionerStarted = 0;
//    int commissionerJoinerAdded = 0;

    while (!PlatformPseudoResetWasRequested())
    {
        otTaskletsProcess(sInstance);
        PlatformProcessDrivers(sInstance);
//		otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_API, "Some msg %s sdd", "dsdsds");

//		uint32_t timeout = 600;
//
//		if (commissionerJoinerAdded == 1) {}
//		else if (threadStarted == 0) {
//        	if (otIp6IsEnabled(sInstance)) {
//        		otThreadSetEnabled(sInstance, true);
//        		threadStarted = 1;
//        	}
//        }
//        else if (commissionerStarted == 0) {
//			if (otThreadGetDeviceRole(sInstance) == OT_DEVICE_ROLE_LEADER) {
//				otCommissionerStart(sInstance);
//				commissionerStarted = 1;
//			}
//		}
//		else if (commissionerJoinerAdded == 0) {
//			if (otCommissionerGetState(sInstance) == OT_COMMISSIONER_STATE_ACTIVE) {
//				otExtAddress addr;
//				const otExtAddress *addrPtr = &addr;
				// Hex2Bin("062cf9a900124b00", addr.m8, sizeof(addr));
//
//				otCommissionerAddJoiner(sInstance, NULL, "WAT2018", timeout);
//				commissionerJoinerAdded = 1;
//
//				HWREG(GPIO_D_BASE + GPIO_O_DIR) |= GPIO_PIN_5;
//				HWREG(GPIO_D_BASE + ((GPIO_PIN_5) << 2)) = 0xFF;
//			}
//        }
//		else if (commissionerJoinerAdded < 2) {
//			if (otCommissionerGetState(sInstance) == OT_COMMISSIONER_STATE_ACTIVE) {
//				otExtAddress addr;
//				const otExtAddress *addrPtr = &addr;
//				Hex2Bin("062cf86200124b00", addr.m8, sizeof(addr));
//
//				otCommissionerAddJoiner(sInstance, addrPtr, "WAT2018", timeout);
//
//				commissionerJoinerAdded = 2;
//			}
//		}
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
