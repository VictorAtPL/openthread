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

#include <assert.h>
#include <openthread-core-config.h>
#include <openthread/config.h>

#include <openthread/cli.h>
#include <openthread/diag.h>
#include <openthread/openthread.h>
#include <openthread/platform/logging.h>

// PIOTR PODBIELSKI
#include "rangeext.h"

#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_gpio.h"
#include <source/gptimer.h>
#include <source/gpio.h>
#include <source/interrupt.h>
#include <source/sys_ctrl.h>
#include <source/gptimer.h>

#include <openthread/message.h>
#include <openthread/udp.h>

#include <string.h>
// PIOTR PODBIELSKI

#include "platform.h"

uint16_t HostSwap16(uint16_t v)
{
    return (((v & 0x00ffU) << 8) & 0xff00) | (((v & 0xff00U) >> 8) & 0x00ff);
}

void otTaskletsSignalPending(otInstance *aInstance)
{
    (void)aInstance;
}

static volatile uint32_t g_ui32Counter = 0;


otInstance *sInstance;
otUdpSocket mSocket;

void HandleUdpReceive(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
	(void)aContext;
	uint8_t buf[1500];
	int     length;

	otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_API, "%d bytes from ", otMessageGetLength(aMessage) - otMessageGetOffset(aMessage));
	otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_API, "%x:%x:%x:%x:%x:%x:%x:%x %d ", HostSwap16(aMessageInfo->mPeerAddr.mFields.m16[0]),
		HostSwap16(aMessageInfo->mPeerAddr.mFields.m16[1]), HostSwap16(aMessageInfo->mPeerAddr.mFields.m16[2]),
		HostSwap16(aMessageInfo->mPeerAddr.mFields.m16[3]), HostSwap16(aMessageInfo->mPeerAddr.mFields.m16[4]),
		HostSwap16(aMessageInfo->mPeerAddr.mFields.m16[5]), HostSwap16(aMessageInfo->mPeerAddr.mFields.m16[6]),
		HostSwap16(aMessageInfo->mPeerAddr.mFields.m16[7]), aMessageInfo->mPeerPort);

	length      = otMessageRead(aMessage, otMessageGetOffset(aMessage), buf, sizeof(buf) - 1);
	buf[length] = '\0';

	otUdpSend(&mSocket, aMessage, aMessageInfo);
}

void
Timer0BIntHandler(void)
{
	//
	// Clear the timer interrupt flag.
	//
	TimerIntClear(GPTIMER0_BASE, GPTIMER_TIMB_TIMEOUT);

	//
	// Update the periodic interrupt counter.
	//
	g_ui32Counter++;

	if (g_ui32Counter % 2 == 0) {
		// check Thread network
		if (otThreadGetDeviceRole(sInstance) == OT_DEVICE_ROLE_CHILD) {
		    IntDisable(INT_TIMER0B);
		    TimerIntDisable(GPTIMER0_BASE, GPTIMER_TIMB_TIMEOUT);
		    TimerIntClear(GPTIMER0_BASE, GPTIMER_TIMB_TIMEOUT);

		    GPIOPinWrite(GPIO_D_BASE, 1, 0x00);

		    otNetifAddress *tmp_otNetifAddess = (otNetifAddress*)otIp6GetUnicastAddresses(sInstance);

		    while (tmp_otNetifAddess != NULL) {
		    	if (tmp_otNetifAddess->mAddress.mFields.m8[0] == 0xFD) {
		    		break;
		    	}

		    	tmp_otNetifAddess = tmp_otNetifAddess->mNext;
		    }

		    otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_API, "Address obtained from otIp6GetUnicastAddresses:");
		    otPlatLog(OT_LOG_LEVEL_DEBG, OT_LOG_REGION_API, "%x:%x:%x:%x:%x:%x:%x:%x", HostSwap16(tmp_otNetifAddess->mAddress.mFields.m16[0]),
		    				HostSwap16(tmp_otNetifAddess->mAddress.mFields.m16[1]), HostSwap16(tmp_otNetifAddess->mAddress.mFields.m16[2]),
		    				HostSwap16(tmp_otNetifAddess->mAddress.mFields.m16[3]), HostSwap16(tmp_otNetifAddess->mAddress.mFields.m16[4]),
		    				HostSwap16(tmp_otNetifAddess->mAddress.mFields.m16[5]), HostSwap16(tmp_otNetifAddess->mAddress.mFields.m16[6]),
		    				HostSwap16(tmp_otNetifAddess->mAddress.mFields.m16[7]));

		    otSockAddr sockaddr;
		    memcpy(&sockaddr.mAddress, &tmp_otNetifAddess->mAddress, sizeof(otIp6Address));
		    sockaddr.mPort    = 6000;
			sockaddr.mScopeId = OT_NETIF_INTERFACE_ID_THREAD;

			otUdpBind(&mSocket, &sockaddr);
		    otUdpOpen(sInstance, &mSocket, HandleUdpReceive, (void*)NULL);

		}
	}
	if (g_ui32Counter % 4 == 0) {
	    GPIOPinWrite(GPIO_D_BASE, 1, 0x00);
	}
	else if (g_ui32Counter % 2 == 0) {
		GPIOPinWrite(GPIO_D_BASE, 1, 0xFF);
	}
}

int main(int argc, char *argv[])
{
//    SysCtrlIOClockSet(SYS_CTRL_SYSDIV_32MHZ);
//    SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_GPT0);
//
//    TimerConfigure(GPTIMER0_BASE, GPTIMER_CFG_SPLIT_PAIR |
//                       GPTIMER_CFG_A_ONE_SHOT | GPTIMER_CFG_B_PERIODIC);
//
//    TimerPrescaleSet(GPTIMER0_BASE, GPTIMER_B, 255);
//
//    TimerLoadSet(GPTIMER0_BASE, GPTIMER_B, SysCtrlClockGet() / 255);
//    TimerIntRegister(GPTIMER0_BASE, GPTIMER_B, Timer0BIntHandler);
//    IntMasterEnable();
//
//    GPIODirModeSet(GPIO_D_BASE, 1, GPIO_DIR_MODE_OUT);



pseudo_reset:
    PlatformInit(argc, argv);

//    g_ui32Counter = 0;
//    TimerIntEnable(GPTIMER0_BASE, GPTIMER_TIMB_TIMEOUT);
//    IntEnable(INT_TIMER0B);
//    GPIOPinWrite(GPIO_D_BASE, 1, 0x00);


    sInstance = otInstanceInitSingle();
    assert(sInstance);

    otCliUartInit(sInstance);
//    TimerEnable(GPTIMER0_BASE, GPTIMER_B);

    while (!PlatformPseudoResetWasRequested())
    {
        otTaskletsProcess(sInstance);
        PlatformProcessDrivers(sInstance);
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
