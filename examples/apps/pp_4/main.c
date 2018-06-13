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
#include <string.h>
#include <stdio.h>
#include <openthread-core-config.h>
#include <openthread/config.h>

#include <openthread/cli.h>
#include <openthread/diag.h>
#include <openthread/openthread.h>
#include <openthread/platform/logging.h>
#include <openthread/message.h>
#include <openthread/udp.h>
#include <openthread/coap.h>

#include "platform.h"
#include "rangeext.h"

#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_gpio.h"
#include <source/gptimer.h>
#include <source/gpio.h>
#include <source/interrupt.h>
#include <source/sys_ctrl.h>
#include <source/gptimer.h>
#include <source/adc.h>

#if OPENTHREAD_ENABLE_MULTIPLE_INSTANCES
void *otPlatCAlloc(size_t aNum, size_t aSize)
{
    return calloc(aNum, aSize);
}

void otPlatFree(void *aPtr)
{
    free(aPtr);
}
#endif

void otTaskletsSignalPending(otInstance *aInstance)
{
    (void)aInstance;
}

int Hex2Bin(const char *aHex, uint8_t *aBin, uint16_t aBinLength)
{
    size_t      hexLength = strlen(aHex);
    const char *hexEnd    = aHex + hexLength;
    uint8_t *   cur       = aBin;
    uint8_t     numChars  = hexLength & 1;
    uint8_t     byte      = 0;

    if ((hexLength + 1) / 2 > aBinLength)
    {
        return -1;
    }

    while (aHex < hexEnd)
    {
        if ('A' <= *aHex && *aHex <= 'F')
        {
            byte |= 10 + (*aHex - 'A');
        }
        else if ('a' <= *aHex && *aHex <= 'f')
        {
            byte |= 10 + (*aHex - 'a');
        }
        else if ('0' <= *aHex && *aHex <= '9')
        {
            byte |= *aHex - '0';
        }
        else
        {
            return -1;
        }

        aHex++;
        numChars++;

        if (numChars >= 2)
        {
            numChars = 0;
            *cur++   = byte;
            byte     = 0;
        }
        else
        {
            byte <<= 4;
        }
    }

    return (int)(cur - aBin);
}

uint16_t HostSwap16(uint16_t v)
{
    return (((v & 0x00ffU) << 8) & 0xff00) | (((v & 0xff00U) >> 8) & 0x00ff);

}

static volatile uint32_t g_ui32Counter = 0;
static volatile otInstance *sInstanceGlobal;
static volatile otUdpSocket mSocketGlobal;
static volatile otSockAddr sockaddrGlobal;

void HandleUdpReceive(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
    (void) aContext;
    uint8_t buf[1500];
    int length;

    otInstance *sInstance = (otInstance *) sInstanceGlobal;

    length = otMessageRead(aMessage, otMessageGetOffset(aMessage), buf, sizeof(buf) - 1);
    buf[length] = '\0';

    otMessageInfo newMessageInfo;

    memcpy(&newMessageInfo.mSockAddr, (otSockAddr *)&sockaddrGlobal, sizeof(otIp6Address));
    memcpy(&newMessageInfo.mPeerAddr, (otIp6Address *)&aMessageInfo->mPeerAddr, sizeof(otIp6Address));
    newMessageInfo.mPeerPort = aMessageInfo->mPeerPort;
    newMessageInfo.mInterfaceId = OT_NETIF_INTERFACE_ID_THREAD;

    otMessage *newMessage = otUdpNewMessage(sInstance, true);
    if (newMessage != NULL) {
        otPlatLog(OT_LOG_LEVEL_INFO, OT_LOG_REGION_API, "Successfully constructed new message");
    }

    if (otMessageAppend(newMessage, buf, length) == OT_ERROR_NONE) {
        otPlatLog(OT_LOG_LEVEL_INFO, OT_LOG_REGION_API, "Successfully appended message");
    }

    if (otUdpSend((otUdpSocket *)&mSocketGlobal, newMessage, &newMessageInfo) == OT_ERROR_NONE) {
        otPlatLog(OT_LOG_LEVEL_INFO, OT_LOG_REGION_API, "Successfully send message");
    }
}

void Timer0BIntHandler(void)
{
    otInstance *sInstance = (otInstance *) sInstanceGlobal;

    TimerIntClear(GPTIMER0_BASE, GPTIMER_TIMB_TIMEOUT);
    g_ui32Counter++;

    if (g_ui32Counter % 2 == 0) { // do every one second
        if (otThreadGetDeviceRole(sInstance) == OT_DEVICE_ROLE_CHILD) {
            IntDisable(INT_TIMER0B);
            TimerIntDisable(GPTIMER0_BASE, GPTIMER_TIMB_TIMEOUT);
            TimerIntClear(GPTIMER0_BASE, GPTIMER_TIMB_TIMEOUT);

            // look for mesh local EID
            otNetifAddress *tmp_otNetifAddess = (otNetifAddress *) otIp6GetUnicastAddresses(sInstance);

            while (tmp_otNetifAddess != NULL) {
                if (tmp_otNetifAddess->mAddress.mFields.m8[0] == 0xFD && !tmp_otNetifAddess->mRloc) {
                    otPlatLog(OT_LOG_LEVEL_INFO, OT_LOG_REGION_API, "Found 0xFD");
                    break;
                }

                tmp_otNetifAddess = tmp_otNetifAddess->mNext;
            }

            // prepare UDP 6000
            memcpy((otIp6Address *)&sockaddrGlobal.mAddress, &tmp_otNetifAddess->mAddress, sizeof(otIp6Address));
            sockaddrGlobal.mPort = 6000;
            sockaddrGlobal.mScopeId = OT_NETIF_INTERFACE_ID_THREAD;

            if (otUdpOpen(sInstance, (otUdpSocket*)&mSocketGlobal, HandleUdpReceive, (void *) NULL) ==
                OT_ERROR_NONE) {
                otPlatLog(OT_LOG_LEVEL_INFO, OT_LOG_REGION_API, "Successfully opened udp");
            }

            if (otUdpBind((otUdpSocket *)&mSocketGlobal, (otSockAddr *)&sockaddrGlobal) == OT_ERROR_NONE) {
                otPlatLog(OT_LOG_LEVEL_INFO, OT_LOG_REGION_API, "Successfully bind udp");
            }
        }
    }
}

int main(int argc, char *argv[])
{
    otInstance *sInstance;

#if OPENTHREAD_ENABLE_MULTIPLE_INSTANCES
    size_t   otInstanceBufferLength = 0;
    uint8_t *otInstanceBuffer       = NULL;
#endif

    SysCtrlIOClockSet(SYS_CTRL_SYSDIV_32MHZ);
    SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_GPT0);

    TimerConfigure(GPTIMER0_BASE, GPTIMER_CFG_SPLIT_PAIR |
                                  GPTIMER_CFG_A_ONE_SHOT | GPTIMER_CFG_B_PERIODIC);

    TimerPrescaleSet(GPTIMER0_BASE, GPTIMER_B, 255);

    TimerLoadSet(GPTIMER0_BASE, GPTIMER_B, SysCtrlClockGet() / 255);
    TimerIntRegister(GPTIMER0_BASE, GPTIMER_B, Timer0BIntHandler);
    IntMasterEnable();

    GPIODirModeSet(GPIO_D_BASE, 1, GPIO_DIR_MODE_OUT);

pseudo_reset:

    PlatformInit(argc, argv);

    g_ui32Counter = 0;
    TimerIntEnable(GPTIMER0_BASE, GPTIMER_TIMB_TIMEOUT);
    IntEnable(INT_TIMER0B);

#if OPENTHREAD_ENABLE_MULTIPLE_INSTANCES
    // Call to query the buffer size
    (void)otInstanceInit(NULL, &otInstanceBufferLength);

    // Call to allocate the buffer
    otInstanceBuffer = (uint8_t *)malloc(otInstanceBufferLength);
    assert(otInstanceBuffer);

    // Initialize OpenThread with the buffer
    sInstance = otInstanceInit(otInstanceBuffer, &otInstanceBufferLength);
#else
    sInstance = otInstanceInitSingle();
#endif
    sInstanceGlobal = (volatile otInstance *) sInstance;
    assert(sInstance);

    otPlatRadioSetTransmitPower(sInstance, -5);

    otExtAddress extAddr;
    Hex2Bin("00124B00062CF84D", extAddr.m8, OT_EXT_ADDRESS_SIZE);
    otLinkFilterAddAddress(sInstance, &extAddr);
    otLinkFilterSetAddressMode(sInstance, OT_MAC_FILTER_ADDRESS_MODE_BLACKLIST);

    otCliUartInit(sInstance);
    TimerEnable(GPTIMER0_BASE, GPTIMER_B);

#if OPENTHREAD_ENABLE_DIAG
    otDiagInit(sInstance);
#endif

    while (!PlatformPseudoResetWasRequested())
    {
        otTaskletsProcess(sInstance);
        PlatformProcessDrivers(sInstance);
    }

    otInstanceFinalize(sInstance);
#if OPENTHREAD_ENABLE_MULTIPLE_INSTANCES
    free(otInstanceBuffer);
#endif

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
