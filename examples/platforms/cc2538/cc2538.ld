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

/**
 * @file
 *   GCC linker script for CC2538.
 */

_512k_bytes             = (512*1024);
_256k_bytes             = (256*1024);
_128k_bytes             = (128*1024);
_FLASH_page_size        = 2048;

/*
 * Change for your chip, default is 512k chips
 */
_FLASH_size_bytes       = _512k_bytes;
_FLASH_n_pages          = (_FLASH_size_bytes / _FLASH_page_size);
/* reduce the usable size by: the CCA + settings Page A & B, total 3 pages */
_FLASH_usable_size      = (_FLASH_size_bytes - (3 * _FLASH_page_size));
_FLASH_start            = 0x00200000;
_FLASH_end              = (_FLASH_start + _FLASH_size_bytes);

/*
 * The CCA (Customer Configuration Area) is always the last page.
 * See: http://www.ti.com/lit/ug/swru319c/swru319c.pdf
 * table 8-2 for more details.
 */
_FLASH_cca_page         = (_FLASH_end - (1 * _FLASH_page_size));

/*
 * OpenThread NV storage goes in the settings page.
 * OpenThread requires at least 2 adjacent pages, call them A and B.
 */
_FLASH_settings_pageB   = (_FLASH_end - (2 * _FLASH_page_size));
_FLASH_settings_pageA   = (_FLASH_end - (3 * _FLASH_page_size));

MEMORY
{
  /* would like to use SYMBOLS (from above)here but we cannot
   * GCC version 4.9 does not support symbolic expressions here.
   * But later versions do support the feature.
   */
  FLASH (rx) :      ORIGIN = 0x00200000,     LENGTH = 0x0007c000
  FLASH_CCA (rx) :  ORIGIN = 0x0027FFD4,     LENGTH = 0x2c
  SRAM (rwx) :      ORIGIN = 0x20000000,     LENGTH = 32K
  PERIPHERAL (rw) : ORIGIN = 0x40000000,     LENGTH = 0x20000000
  SYSPERIPH (rw) :  ORIGIN = 0xe0000000,     LENGTH = 0x00042000
}
/*
 * To safty check what would have been the SYMBOL values
 * we use these ASSERTS to verify things are still good.
 */
ASSERT( _FLASH_start       == 0x00200000, "invalid flash start address for cc2538")
ASSERT( _FLASH_cca_page    == 0x0027f800, "invalid cca start address for cc2538")
ASSERT( _FLASH_usable_size == 0x0007e800, "Invalid usable size for this config")



ENTRY(flash_cca_lock_page)
SECTIONS
{
    .text : ALIGN(4)
    {
        _text = .;
        *(.vectors)
        *(.text*)
        *(.rodata*)
        KEEP(*(.init))
        KEEP(*(.fini))
        _etext = .;
    } > FLASH= 0

    .init_array :
    {
        _init_array = .;
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array*))
        _einit_array = .;
    } > FLASH

    .ARM.exidx : ALIGN(4)
    {
        *(.ARM.exidx*)
    } > FLASH

    .data : ALIGN(4)
    {
        _data = .;
        *(.data*)
        _edata = .;
    } > SRAM AT > FLASH
    _ldata = LOADADDR(.data);

    .bss : ALIGN(4)
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > SRAM

    _heap = .;
    end = .;

    .stack : ALIGN(4)
    {
        *(.stack)
    } > SRAM

    .flashcca :
    {
        KEEP(*(.flash_cca))
    } > FLASH_CCA
    
    /* Peripheral base addresses */
    .peripheral : ALIGN(4)
    {
        /*
         * Addresses are relative to the start of
         * the PERIPHERAL block (0x40000000)
         */
        SSI0_BASE   = 0x00008000;
 
        SSI0_CR0            = SSI0_BASE + 0x000;
        SSI0_CR1            = SSI0_BASE + 0x004;
        SSI0_DR             = SSI0_BASE + 0x008;
        SSI0_SR             = SSI0_BASE + 0x00c;
        SSI0_CPSR           = SSI0_BASE + 0x010;
        SSI0_IM             = SSI0_BASE + 0x014;
        SSI0_RIS            = SSI0_BASE + 0x018;
        SSI0_MIS            = SSI0_BASE + 0x01c;
        SSI0_ICR            = SSI0_BASE + 0x020;
        SSI0_DMACTRL            = SSI0_BASE + 0x024;
        SSI0_CC             = SSI0_BASE + 0xfc8;
 
        SSI1_BASE   = 0x00009000;
 
        SSI1_CR0            = SSI1_BASE + 0x000;
        SSI1_CR1            = SSI1_BASE + 0x004;
        SSI1_DR             = SSI1_BASE + 0x008;
        SSI1_SR             = SSI1_BASE + 0x00c;
        SSI1_CPSR           = SSI1_BASE + 0x010;
        SSI1_IM             = SSI1_BASE + 0x014;
        SSI1_RIS            = SSI1_BASE + 0x018;
        SSI1_MIS            = SSI1_BASE + 0x01c;
        SSI1_ICR            = SSI1_BASE + 0x020;
        SSI1_DMACTRL            = SSI1_BASE + 0x024;
        SSI1_CC             = SSI1_BASE + 0xfc8;
 
        UART0_BASE  = 0x0000c000;
 
        UART0_DR            = UART0_BASE + 0x000;
        UART0_RSR           = UART0_BASE + 0x004;
        UART0_ECR           = UART0_BASE + 0x004;
        UART0_FR            = UART0_BASE + 0x018;
        UART0_IPLR          = UART0_BASE + 0x020;
        UART0_IBRD          = UART0_BASE + 0x024;
        UART0_FBRD          = UART0_BASE + 0x028;
        UART0_LCRH          = UART0_BASE + 0x02c;
        UART0_CTL           = UART0_BASE + 0x030;
        UART0_IFLS          = UART0_BASE + 0x034;
        UART0_IM            = UART0_BASE + 0x038;
        UART0_RIS           = UART0_BASE + 0x03c;
        UART0_MIS           = UART0_BASE + 0x040;
        UART0_ICR           = UART0_BASE + 0x044;
        UART0_DMACTL            = UART0_BASE + 0x048;
        UART0_LCTL          = UART0_BASE + 0x090;
        UART0_LSS           = UART0_BASE + 0x094;
        UART0_LTIM          = UART0_BASE + 0x098;
        UART0_NINEBITADDR       = UART0_BASE + 0x0a4;
        UART0_NINEBITMASK       = UART0_BASE + 0x0a8;
        UART0_PP            = UART0_BASE + 0xfc0;
        UART0_CC            = UART0_BASE + 0xfc8;
 
        UART1_BASE  = 0x0000d000;
 
        UART1_DR            = UART1_BASE + 0x000;
        UART1_RSR           = UART1_BASE + 0x004;
        UART1_ECR           = UART1_BASE + 0x004;
        UART1_FR            = UART1_BASE + 0x018;
        UART1_IPLR          = UART1_BASE + 0x020;
        UART1_IBRD          = UART1_BASE + 0x024;
        UART1_FBRD          = UART1_BASE + 0x028;
        UART1_LCRH          = UART1_BASE + 0x02c;
        UART1_CTL           = UART1_BASE + 0x030;
        UART1_IFLS          = UART1_BASE + 0x034;
        UART1_IM            = UART1_BASE + 0x038;
        UART1_RIS           = UART1_BASE + 0x03c;
        UART1_MIS           = UART1_BASE + 0x040;
        UART1_ICR           = UART1_BASE + 0x044;
        UART1_DMACTL            = UART1_BASE + 0x048;
        UART1_LCTL          = UART1_BASE + 0x090;
        UART1_LSS           = UART1_BASE + 0x094;
        UART1_LTIM          = UART1_BASE + 0x098;
        UART1_NINEBITADDR       = UART1_BASE + 0x0a4;
        UART1_NINEBITMASK       = UART1_BASE + 0x0a8;
        UART1_PP            = UART1_BASE + 0xfc0;
        UART1_CC            = UART1_BASE + 0xfc8;
 
        I2CM_BASE   = 0x00020000;
 
        I2CM_SA             = I2CM_BASE + 0x000;
        I2CM_CTRL           = I2CM_BASE + 0x004;
        I2CM_STAT           = I2CM_BASE + 0x004;
        I2CM_DR             = I2CM_BASE + 0x008;
        I2CM_TPR            = I2CM_BASE + 0x00c;
        I2CM_IMR            = I2CM_BASE + 0x010;
        I2CM_RIS            = I2CM_BASE + 0x014;
        I2CM_MIS            = I2CM_BASE + 0x018;
        I2CM_ICR            = I2CM_BASE + 0x01c;
        I2CM_CR             = I2CM_BASE + 0x020;
 
        I2CS_BASE   = 0x00020800;
 
        I2CS_OAR            = I2CS_BASE + 0x000;
        I2CS_STAT           = I2CS_BASE + 0x004;
        I2CS_CTRL           = I2CS_BASE + 0x004;
        I2CS_DR             = I2CS_BASE + 0x008;
        I2CS_IMR            = I2CS_BASE + 0x00c;
        I2CS_RIS            = I2CS_BASE + 0x010;
        I2CS_MIS            = I2CS_BASE + 0x014;
        I2CS_ICR            = I2CS_BASE + 0x018;
 
        GPT0_BASE   = 0x00030000;
        GPT_SYNC            = GPT0_BASE + 0x010;
 
        GPT0_CFG            = GPT0_BASE + 0x000;
        GPT0_TAMR           = GPT0_BASE + 0x004;
        GPT0_TBMR           = GPT0_BASE + 0x008;
        GPT0_CTL            = GPT0_BASE + 0x00c;
        GPT0_IMR            = GPT0_BASE + 0x018;
        GPT0_RIS            = GPT0_BASE + 0x01c;
        GPT0_MIS            = GPT0_BASE + 0x020;
        GPT0_ICR            = GPT0_BASE + 0x024;
        GPT0_TAILR          = GPT0_BASE + 0x028;
        GPT0_TBILR          = GPT0_BASE + 0x02c;
        GPT0_TAMATCHR           = GPT0_BASE + 0x030;
        GPT0_TBMATCHR           = GPT0_BASE + 0x034;
        GPT0_TAPR           = GPT0_BASE + 0x038;
        GPT0_TBPR           = GPT0_BASE + 0x03c;
        GPT0_TAPMR          = GPT0_BASE + 0x040;
        GPT0_TBPMR          = GPT0_BASE + 0x044;
        GPT0_TAR            = GPT0_BASE + 0x048;
        GPT0_TBR            = GPT0_BASE + 0x04c;
        GPT0_TAV            = GPT0_BASE + 0x050;
        GPT0_TBV            = GPT0_BASE + 0x054;
        GPT0_TAPS           = GPT0_BASE + 0x05c;
        GPT0_TBPS           = GPT0_BASE + 0x060;
        GPT0_TAPV           = GPT0_BASE + 0x064;
        GPT0_TBPV           = GPT0_BASE + 0x068;
        GPT0_PP             = GPT0_BASE + 0xfc0;
 
        GPT1_BASE   = 0x00031000;
 
        GPT1_CFG            = GPT1_BASE + 0x000;
        GPT1_TAMR           = GPT1_BASE + 0x004;
        GPT1_TBMR           = GPT1_BASE + 0x008;
        GPT1_CTL            = GPT1_BASE + 0x00c;
        GPT1_IMR            = GPT1_BASE + 0x018;
        GPT1_RIS            = GPT1_BASE + 0x01c;
        GPT1_MIS            = GPT1_BASE + 0x020;
        GPT1_ICR            = GPT1_BASE + 0x024;
        GPT1_TAILR          = GPT1_BASE + 0x028;
        GPT1_TBILR          = GPT1_BASE + 0x02c;
        GPT1_TAMATCHR           = GPT1_BASE + 0x030;
        GPT1_TBMATCHR           = GPT1_BASE + 0x034;
        GPT1_TAPR           = GPT1_BASE + 0x038;
        GPT1_TBPR           = GPT1_BASE + 0x03c;
        GPT1_TAPMR          = GPT1_BASE + 0x040;
        GPT1_TBPMR          = GPT1_BASE + 0x044;
        GPT1_TAR            = GPT1_BASE + 0x048;
        GPT1_TBR            = GPT1_BASE + 0x04c;
        GPT1_TAV            = GPT1_BASE + 0x050;
        GPT1_TBV            = GPT1_BASE + 0x054;
        GPT1_TAPS           = GPT1_BASE + 0x05c;
        GPT1_TBPS           = GPT1_BASE + 0x060;
        GPT1_TAPV           = GPT1_BASE + 0x064;
        GPT1_TBPV           = GPT1_BASE + 0x068;
        GPT1_PP             = GPT1_BASE + 0xfc0;
 
        GPT2_BASE   = 0x00032000;
 
        GPT2_CFG            = GPT2_BASE + 0x000;
        GPT2_TAMR           = GPT2_BASE + 0x004;
        GPT2_TBMR           = GPT2_BASE + 0x008;
        GPT2_CTL            = GPT2_BASE + 0x00c;
        GPT2_IMR            = GPT2_BASE + 0x018;
        GPT2_RIS            = GPT2_BASE + 0x01c;
        GPT2_MIS            = GPT2_BASE + 0x020;
        GPT2_ICR            = GPT2_BASE + 0x024;
        GPT2_TAILR          = GPT2_BASE + 0x028;
        GPT2_TBILR          = GPT2_BASE + 0x02c;
        GPT2_TAMATCHR           = GPT2_BASE + 0x030;
        GPT2_TBMATCHR           = GPT2_BASE + 0x034;
        GPT2_TAPR           = GPT2_BASE + 0x038;
        GPT2_TBPR           = GPT2_BASE + 0x03c;
        GPT2_TAPMR          = GPT2_BASE + 0x040;
        GPT2_TBPMR          = GPT2_BASE + 0x044;
        GPT2_TAR            = GPT2_BASE + 0x048;
        GPT2_TBR            = GPT2_BASE + 0x04c;
        GPT2_TAV            = GPT2_BASE + 0x050;
        GPT2_TBV            = GPT2_BASE + 0x054;
        GPT2_TAPS           = GPT2_BASE + 0x05c;
        GPT2_TBPS           = GPT2_BASE + 0x060;
        GPT2_TAPV           = GPT2_BASE + 0x064;
        GPT2_TBPV           = GPT2_BASE + 0x068;
        GPT2_PP             = GPT2_BASE + 0xfc0;
 
        GPT3_BASE   = 0x00033000;
 
        GPT3_CFG            = GPT3_BASE + 0x000;
        GPT3_TAMR           = GPT3_BASE + 0x004;
        GPT3_TBMR           = GPT3_BASE + 0x008;
        GPT3_CTL            = GPT3_BASE + 0x00c;
        GPT3_IMR            = GPT3_BASE + 0x018;
        GPT3_RIS            = GPT3_BASE + 0x01c;
        GPT3_MIS            = GPT3_BASE + 0x020;
        GPT3_ICR            = GPT3_BASE + 0x024;
        GPT3_TAILR          = GPT3_BASE + 0x028;
        GPT3_TBILR          = GPT3_BASE + 0x02c;
        GPT3_TAMATCHR           = GPT3_BASE + 0x030;
        GPT3_TBMATCHR           = GPT3_BASE + 0x034;
        GPT3_TAPR           = GPT3_BASE + 0x038;
        GPT3_TBPR           = GPT3_BASE + 0x03c;
        GPT3_TAPMR          = GPT3_BASE + 0x040;
        GPT3_TBPMR          = GPT3_BASE + 0x044;
        GPT3_TAR            = GPT3_BASE + 0x048;
        GPT3_TBR            = GPT3_BASE + 0x04c;
        GPT3_TAV            = GPT3_BASE + 0x050;
        GPT3_TBV            = GPT3_BASE + 0x054;
        GPT3_TAPS           = GPT3_BASE + 0x05c;
        GPT3_TBPS           = GPT3_BASE + 0x060;
        GPT3_TAPV           = GPT3_BASE + 0x064;
        GPT3_TBPV           = GPT3_BASE + 0x068;
        GPT3_PP             = GPT3_BASE + 0xfc0;
 
        RFCORE_BASE = 0x00088000;
 
        RFCORE_FFSM_SRCADDRESS_TABLE    = RFCORE_BASE + 0x400;
 
        RFCORE_FFSM_SRCEXTPENDEN0   = RFCORE_BASE + 0x590;
        RFCORE_FFSM_SRCSHORTPENDEN0 = RFCORE_BASE + 0x59c;
        RFCORE_FFSM_EXT_ADDR0       = RFCORE_BASE + 0x5a8;
        RFCORE_FFSM_PAN_ID0     = RFCORE_BASE + 0x5c8;
        RFCORE_FFSM_PAN_ID1     = RFCORE_BASE + 0x5cc;
        RFCORE_FFSM_SHORT_ADDR0     = RFCORE_BASE + 0x5d0;
        RFCORE_FFSM_SHORT_ADDR1     = RFCORE_BASE + 0x5d4;
        RFCORE_XREG_FRMFILT0        = RFCORE_BASE + 0x600;
        RFCORE_XREG_SRCMATCH        = RFCORE_BASE + 0x608;
        RFCORE_XREG_SRCSHORTEN0     = RFCORE_BASE + 0x60c;
        RFCORE_XREG_SRCEXTEN0       = RFCORE_BASE + 0x618;
 
        RFCORE_XREG_FRMCTRL0        = RFCORE_BASE + 0x624;
        RFCORE_XREG_FRMCTRL1        = RFCORE_BASE + 0x628;
        RFCORE_XREG_RXENABLE        = RFCORE_BASE + 0x62c;
        RFCORE_XREG_FREQCTRL        = RFCORE_BASE + 0x63c;
        RFCORE_XREG_TXPOWER     = RFCORE_BASE + 0x640;
        RFCORE_XREG_FSMSTAT1        = RFCORE_BASE + 0x64c;
        RFCORE_XREG_FIFOPCTRL       = RFCORE_BASE + 0x650;
        RFCORE_XREG_CCACTRL0        = RFCORE_BASE + 0x658;
        RFCORE_XREG_RSSISTAT        = RFCORE_BASE + 0x664;
        RFCORE_XREG_AGCCTRL1        = RFCORE_BASE + 0x6c8;
        RFCORE_XREG_RFC_OBS_CTRL    = RFCORE_BASE + 0x7ac;
        RFCORE_XREG_TXFILTCFG       = RFCORE_BASE + 0x7e8;
        RFCORE_XREG_RFRND       = RFCORE_BASE + 0x69c;
        RFCORE_SFR_RFDATA       = RFCORE_BASE + 0x828;
        RFCORE_SFR_RFERRF       = RFCORE_BASE + 0x82c;
        RFCORE_SFR_RFIRQF0      = RFCORE_BASE + 0x834;
        RFCORE_SFR_RFST         = RFCORE_BASE + 0x838;
 
        USB_BASE    = 0x00089000;
        AES_BASE    = 0x0008b000;
        AES_DMAC_CH0_CTRL       = AES_BASE + 0x000;
        AES_DMAC_CH0_EXTADDR        = AES_BASE + 0x004;
        AES_DMAC_CH0_DMALENGTH      = AES_BASE + 0x00C;
        AES_DMAC_STATUS         = AES_BASE + 0x018;
        AES_DMAC_SWRES          = AES_BASE + 0x01C;
        AES_DMAC_CH1_CTRL       = AES_BASE + 0x020;
        AES_DMAC_CH1_EXTADDR        = AES_BASE + 0x024;
        AES_DMAC_CH1_DMALENGTH      = AES_BASE + 0x02C;
        AES_DMAC_MST_RUNPARAMS      = AES_BASE + 0x078;
        AES_DMAC_PERSR          = AES_BASE + 0x07C;
        AES_DMAC_OPTIONS        = AES_BASE + 0x0F8;
        AES_DMAC_VERSION        = AES_BASE + 0x0FC;
        AES_HASH_IO_BUF_CTRL        = AES_BASE + 0x640;
        AES_HASH_MODE_IN        = AES_BASE + 0x644;
        AES_HASH_LENGTH_IN_L        = AES_BASE + 0x648;
        AES_HASH_LENGTH_IN_H        = AES_BASE + 0x64C;
        AES_HASH_DIGEST_A       = AES_BASE + 0x650;
        AES_HASH_DIGEST_B       = AES_BASE + 0x654;
        AES_HASH_DIGEST_C       = AES_BASE + 0x658;
        AES_HASH_DIGEST_D       = AES_BASE + 0x65C;
        AES_HASH_DIGEST_E       = AES_BASE + 0x660;
        AES_HASH_DIGEST_F       = AES_BASE + 0x664;
        AES_HASH_DIGEST_G       = AES_BASE + 0x668;
        AES_HASH_DIGEST_H       = AES_BASE + 0x66C;
        AES_CTRL_ALG_SEL        = AES_BASE + 0x700;
        AES_CTRL_PROT_EN        = AES_BASE + 0x704;
        AES_CTRL_SW_RESET       = AES_BASE + 0x740;
        AES_CTRL_INT_CFG        = AES_BASE + 0x780;
        AES_CTRL_INT_EN         = AES_BASE + 0x784;
        AES_CTRL_INT_CLR        = AES_BASE + 0x788;
        AES_CTRL_INT_SET        = AES_BASE + 0x78C;
        AES_CTRL_INT_STAT       = AES_BASE + 0x790;
        AES_CTRL_OPTIONS        = AES_BASE + 0x7F8;
 
        SYS_CTRL_BASE   = 0x000d2000;
        SYS_CTRL_CLOCK_CTRL     = SYS_CTRL_BASE + 0x000;
        SYS_CTRL_CLOCK_STA      = SYS_CTRL_BASE + 0x004;
        SYS_CTRL_RCGCGPT        = SYS_CTRL_BASE + 0x008;
        SYS_CTRL_SCGCGPT        = SYS_CTRL_BASE + 0x00c;
        SYS_CTRL_DCGCGPT        = SYS_CTRL_BASE + 0x010;
        SYS_CTRL_SRGPT          = SYS_CTRL_BASE + 0x014;
        SYS_CTRL_RCGCSSI        = SYS_CTRL_BASE + 0x018;
        SYS_CTRL_SCGCSSI        = SYS_CTRL_BASE + 0x01c;
        SYS_CTRL_DCGCSSI        = SYS_CTRL_BASE + 0x020;
        SYS_CTRL_SRSSI          = SYS_CTRL_BASE + 0x024;
        SYS_CTRL_RCGCUART       = SYS_CTRL_BASE + 0x028;
        SYS_CTRL_SCGCUART       = SYS_CTRL_BASE + 0x02c;
        SYS_CTRL_DCGCUART       = SYS_CTRL_BASE + 0x030;
        SYS_CTRL_SRUART         = SYS_CTRL_BASE + 0x034;
        SYS_CTRL_RCGCI2C        = SYS_CTRL_BASE + 0x038;
        SYS_CTRL_SCGCI2C        = SYS_CTRL_BASE + 0x03c;
        SYS_CTRL_DCGCI2C        = SYS_CTRL_BASE + 0x040;
        SYS_CTRL_SRI2C          = SYS_CTRL_BASE + 0x044;
        SYS_CTRL_RCGCSEC        = SYS_CTRL_BASE + 0x048;
        SYS_CTRL_SCGCSEC        = SYS_CTRL_BASE + 0x04c;
        SYS_CTRL_DCGCSEC        = SYS_CTRL_BASE + 0x050;
        SYS_CTRL_SRSEC          = SYS_CTRL_BASE + 0x054;
        SYS_CTRL_PMCTL          = SYS_CTRL_BASE + 0x058;
        SYS_CTRL_SRCRC          = SYS_CTRL_BASE + 0x05c;
        SYS_CTRL_PWRDBG         = SYS_CTRL_BASE + 0x074;
        SYS_CTRL_CLD            = SYS_CTRL_BASE + 0x080;
        SYS_CTRL_IWE            = SYS_CTRL_BASE + 0x094;
        SYS_CTRL_I_MAP          = SYS_CTRL_BASE + 0x098;
        SYS_CTRL_RCGCRFC        = SYS_CTRL_BASE + 0x0a8;
        SYS_CTRL_SCGCRFC        = SYS_CTRL_BASE + 0x0ac;
        SYS_CTRL_DCGCRFC        = SYS_CTRL_BASE + 0x0b0;
        SYS_CTRL_EMUOVR         = SYS_CTRL_BASE + 0x0b4;
 
        FLASH_BASE  = 0x000d3000;
 
        GPIO_IOC_BASE   = 0x000d4000;
 
        /* IOC output selection */
        GPIO_IOC_SEL            = GPIO_IOC_BASE + 0x000;
        GPIO_IOC_SEL_PA         = GPIO_IOC_SEL + 0x00;
        GPIO_IOC_SEL_PA0        = GPIO_IOC_SEL_PA + 0x00;
        GPIO_IOC_SEL_PA1        = GPIO_IOC_SEL_PA + 0x04;
        GPIO_IOC_SEL_PA2        = GPIO_IOC_SEL_PA + 0x08;
        GPIO_IOC_SEL_PA3        = GPIO_IOC_SEL_PA + 0x0c;
        GPIO_IOC_SEL_PA4        = GPIO_IOC_SEL_PA + 0x10;
        GPIO_IOC_SEL_PA5        = GPIO_IOC_SEL_PA + 0x14;
        GPIO_IOC_SEL_PA6        = GPIO_IOC_SEL_PA + 0x18;
        GPIO_IOC_SEL_PA7        = GPIO_IOC_SEL_PA + 0x1c;
        GPIO_IOC_SEL_PB         = GPIO_IOC_SEL + 0x20;
        GPIO_IOC_SEL_PB0        = GPIO_IOC_SEL_PB + 0x00;
        GPIO_IOC_SEL_PB1        = GPIO_IOC_SEL_PB + 0x04;
        GPIO_IOC_SEL_PB2        = GPIO_IOC_SEL_PB + 0x08;
        GPIO_IOC_SEL_PB3        = GPIO_IOC_SEL_PB + 0x0c;
        GPIO_IOC_SEL_PB4        = GPIO_IOC_SEL_PB + 0x10;
        GPIO_IOC_SEL_PB5        = GPIO_IOC_SEL_PB + 0x14;
        GPIO_IOC_SEL_PB6        = GPIO_IOC_SEL_PB + 0x18;
        GPIO_IOC_SEL_PB7        = GPIO_IOC_SEL_PB + 0x1c;
        GPIO_IOC_SEL_PC         = GPIO_IOC_SEL + 0x40;
        GPIO_IOC_SEL_PC0        = GPIO_IOC_SEL_PC + 0x00;
        GPIO_IOC_SEL_PC1        = GPIO_IOC_SEL_PC + 0x04;
        GPIO_IOC_SEL_PC2        = GPIO_IOC_SEL_PC + 0x08;
        GPIO_IOC_SEL_PC3        = GPIO_IOC_SEL_PC + 0x0c;
        GPIO_IOC_SEL_PC4        = GPIO_IOC_SEL_PC + 0x10;
        GPIO_IOC_SEL_PC5        = GPIO_IOC_SEL_PC + 0x14;
        GPIO_IOC_SEL_PC6        = GPIO_IOC_SEL_PC + 0x18;
        GPIO_IOC_SEL_PC7        = GPIO_IOC_SEL_PC + 0x1c;
        GPIO_IOC_SEL_PD         = GPIO_IOC_SEL + 0x60;
        GPIO_IOC_SEL_PD0        = GPIO_IOC_SEL_PD + 0x00;
        GPIO_IOC_SEL_PD1        = GPIO_IOC_SEL_PD + 0x04;
        GPIO_IOC_SEL_PD2        = GPIO_IOC_SEL_PD + 0x08;
        GPIO_IOC_SEL_PD3        = GPIO_IOC_SEL_PD + 0x0c;
        GPIO_IOC_SEL_PD4        = GPIO_IOC_SEL_PD + 0x10;
        GPIO_IOC_SEL_PD5        = GPIO_IOC_SEL_PD + 0x14;
        GPIO_IOC_SEL_PD6        = GPIO_IOC_SEL_PD + 0x18;
        GPIO_IOC_SEL_PD7        = GPIO_IOC_SEL_PD + 0x1c;
 
        /* IOC override selection */
        GPIO_IOC_OVER           = GPIO_IOC_BASE + 0x080;
        GPIO_IOC_OVER_PA        = GPIO_IOC_OVER + 0x00;
        GPIO_IOC_OVER_PA0       = GPIO_IOC_OVER_PA + 0x00;
        GPIO_IOC_OVER_PA1       = GPIO_IOC_OVER_PA + 0x04;
        GPIO_IOC_OVER_PA2       = GPIO_IOC_OVER_PA + 0x08;
        GPIO_IOC_OVER_PA3       = GPIO_IOC_OVER_PA + 0x0c;
        GPIO_IOC_OVER_PA4       = GPIO_IOC_OVER_PA + 0x10;
        GPIO_IOC_OVER_PA5       = GPIO_IOC_OVER_PA + 0x14;
        GPIO_IOC_OVER_PA6       = GPIO_IOC_OVER_PA + 0x18;
        GPIO_IOC_OVER_PA7       = GPIO_IOC_OVER_PA + 0x1c;
        GPIO_IOC_OVER_PB        = GPIO_IOC_OVER + 0x20;
        GPIO_IOC_OVER_PB0       = GPIO_IOC_OVER_PB + 0x00;
        GPIO_IOC_OVER_PB1       = GPIO_IOC_OVER_PB + 0x04;
        GPIO_IOC_OVER_PB2       = GPIO_IOC_OVER_PB + 0x08;
        GPIO_IOC_OVER_PB3       = GPIO_IOC_OVER_PB + 0x0c;
        GPIO_IOC_OVER_PB4       = GPIO_IOC_OVER_PB + 0x10;
        GPIO_IOC_OVER_PB5       = GPIO_IOC_OVER_PB + 0x14;
        GPIO_IOC_OVER_PB6       = GPIO_IOC_OVER_PB + 0x18;
        GPIO_IOC_OVER_PB7       = GPIO_IOC_OVER_PB + 0x1c;
        GPIO_IOC_OVER_PC        = GPIO_IOC_OVER + 0x40;
        GPIO_IOC_OVER_PC0       = GPIO_IOC_OVER_PC + 0x00;
        GPIO_IOC_OVER_PC1       = GPIO_IOC_OVER_PC + 0x04;
        GPIO_IOC_OVER_PC2       = GPIO_IOC_OVER_PC + 0x08;
        GPIO_IOC_OVER_PC3       = GPIO_IOC_OVER_PC + 0x0c;
        GPIO_IOC_OVER_PC4       = GPIO_IOC_OVER_PC + 0x10;
        GPIO_IOC_OVER_PC5       = GPIO_IOC_OVER_PC + 0x14;
        GPIO_IOC_OVER_PC6       = GPIO_IOC_OVER_PC + 0x18;
        GPIO_IOC_OVER_PC7       = GPIO_IOC_OVER_PC + 0x1c;
        GPIO_IOC_OVER_PD        = GPIO_IOC_OVER + 0x60;
        GPIO_IOC_OVER_PD0       = GPIO_IOC_OVER_PD + 0x00;
        GPIO_IOC_OVER_PD1       = GPIO_IOC_OVER_PD + 0x04;
        GPIO_IOC_OVER_PD2       = GPIO_IOC_OVER_PD + 0x08;
        GPIO_IOC_OVER_PD3       = GPIO_IOC_OVER_PD + 0x0c;
        GPIO_IOC_OVER_PD4       = GPIO_IOC_OVER_PD + 0x10;
        GPIO_IOC_OVER_PD5       = GPIO_IOC_OVER_PD + 0x14;
        GPIO_IOC_OVER_PD6       = GPIO_IOC_OVER_PD + 0x18;
        GPIO_IOC_OVER_PD7       = GPIO_IOC_OVER_PD + 0x1c;
 
        /* IOC UART0 control lines */
        GPIO_IOC_UART0_RXD      = GPIO_IOC_BASE + 0x100;
 
        /* IOC UART1 control lines */
        GPIO_IOC_UART1_CTS      = GPIO_IOC_BASE + 0x104;
        GPIO_IOC_UART1_RXD      = GPIO_IOC_BASE + 0x108;
 
        /* IOC SSI0 control lines */
        GPIO_IOC_SSI0_CLK       = GPIO_IOC_BASE + 0x10c;
        GPIO_IOC_SSI0_RXD       = GPIO_IOC_BASE + 0x110;
        GPIO_IOC_SSI0_FSS       = GPIO_IOC_BASE + 0x114;
        GPIO_IOC_SSI0_CLK_SSIIN     = GPIO_IOC_BASE + 0x118;
 
        /* IOC SSI1 control lines */
        GPIO_IOC_SSI1_CLK       = GPIO_IOC_BASE + 0x11c;
        GPIO_IOC_SSI1_RXD       = GPIO_IOC_BASE + 0x120;
        GPIO_IOC_SSI1_FSS       = GPIO_IOC_BASE + 0x124;
        GPIO_IOC_SSI1_CLK_SSIIN     = GPIO_IOC_BASE + 0x128;
 
        /* IOC I²C master control lines */
        GPIO_IOC_I2CM_SDA       = GPIO_IOC_BASE + 0x12c;
        GPIO_IOC_I2CM_SCL       = GPIO_IOC_BASE + 0x130;
 
        /* IOC General Purpose timer control lines */
        GPIO_IOC_GPT0_OCP1      = GPIO_IOC_BASE + 0x134;
        GPIO_IOC_GPT0_OCP2      = GPIO_IOC_BASE + 0x138;
        GPIO_IOC_GPT1_OCP1      = GPIO_IOC_BASE + 0x13c;
        GPIO_IOC_GPT1_OCP2      = GPIO_IOC_BASE + 0x140;
        GPIO_IOC_GPT2_OCP1      = GPIO_IOC_BASE + 0x144;
        GPIO_IOC_GPT2_OCP2      = GPIO_IOC_BASE + 0x148;
        GPIO_IOC_GPT3_OCP1      = GPIO_IOC_BASE + 0x14c;
        GPIO_IOC_GPT3_OCP2      = GPIO_IOC_BASE + 0x150;
 
        WDT_BASE    = 0x000d5000;
        SMWDTHROSC_WDCTL        = WDT_BASE + 0x000;
 
        ANALOGUE_BASE   = 0x000d6000;
        ADC_BASE    = 0x000d7000;
 
        GPIOA_BASE  = 0x000d9000;
        GPIOA_DATA          = GPIOA_BASE + 0x000;
        GPIOA_DIR           = GPIOA_BASE + 0x400;
        GPIOA_IS            = GPIOA_BASE + 0x404;
        GPIOA_IBE           = GPIOA_BASE + 0x408;
        GPIOA_IEV           = GPIOA_BASE + 0x40c;
        GPIOA_IE            = GPIOA_BASE + 0x410;
        GPIOA_RIS           = GPIOA_BASE + 0x414;
        GPIOA_MIS           = GPIOA_BASE + 0x418;
        GPIOA_IC            = GPIOA_BASE + 0x41c;
        GPIOA_AFSEL         = GPIOA_BASE + 0x420;
        GPIOA_GPIOLOCK          = GPIOA_BASE + 0x520;
        GPIOA_GPIOCR            = GPIOA_BASE + 0x524;
        GPIOA_PMUX          = GPIOA_BASE + 0x700;
        GPIOA_P_EDGE_CTRL       = GPIOA_BASE + 0x704;
        GPIOA_PI_EN         = GPIOA_BASE + 0x710;
        GPIOA_IRQ_DETECT_ACK        = GPIOA_BASE + 0x718;
        GPIOA_USB_IRQ_ACK       = GPIOA_BASE + 0x71c;
        GPIOA_IRQ_DETECT_UNMASK     = GPIOA_BASE + 0x720;
 
        GPIOB_BASE  = 0x000da000;
        GPIOB_DATA          = GPIOB_BASE + 0x000;
        GPIOB_DIR           = GPIOB_BASE + 0x400;
        GPIOB_IS            = GPIOB_BASE + 0x404;
        GPIOB_IBE           = GPIOB_BASE + 0x408;
        GPIOB_IEV           = GPIOB_BASE + 0x40c;
        GPIOB_IE            = GPIOB_BASE + 0x410;
        GPIOB_RIS           = GPIOB_BASE + 0x414;
        GPIOB_MIS           = GPIOB_BASE + 0x418;
        GPIOB_IC            = GPIOB_BASE + 0x41c;
        GPIOB_AFSEL         = GPIOB_BASE + 0x420;
        GPIOB_GPIOLOCK          = GPIOB_BASE + 0x520;
        GPIOB_GPIOCR            = GPIOB_BASE + 0x524;
        GPIOB_PMUX          = GPIOB_BASE + 0x700;
        GPIOB_P_EDGE_CTRL       = GPIOB_BASE + 0x704;
        GPIOB_PI_EN         = GPIOB_BASE + 0x710;
        GPIOB_IRQ_DETECT_ACK        = GPIOB_BASE + 0x718;
        GPIOB_USB_IRQ_ACK       = GPIOB_BASE + 0x71c;
        GPIOB_IRQ_DETECT_UNMASK     = GPIOB_BASE + 0x720;
 
        GPIOC_BASE  = 0x000db000;
        GPIOC_DATA          = GPIOC_BASE + 0x000;
        GPIOC_DIR           = GPIOC_BASE + 0x400;
        GPIOC_IS            = GPIOC_BASE + 0x404;
        GPIOC_IBE           = GPIOC_BASE + 0x408;
        GPIOC_IEV           = GPIOC_BASE + 0x40c;
        GPIOC_IE            = GPIOC_BASE + 0x410;
        GPIOC_RIS           = GPIOC_BASE + 0x414;
        GPIOC_MIS           = GPIOC_BASE + 0x418;
        GPIOC_IC            = GPIOC_BASE + 0x41c;
        GPIOC_AFSEL         = GPIOC_BASE + 0x420;
        GPIOC_GPIOLOCK          = GPIOC_BASE + 0x520;
        GPIOC_GPIOCR            = GPIOC_BASE + 0x524;
        GPIOC_PMUX          = GPIOC_BASE + 0x700;
        GPIOC_P_EDGE_CTRL       = GPIOC_BASE + 0x704;
        GPIOC_PI_EN         = GPIOC_BASE + 0x710;
        GPIOC_IRQ_DETECT_ACK        = GPIOC_BASE + 0x718;
        GPIOC_USB_IRQ_ACK       = GPIOC_BASE + 0x71c;
        GPIOC_IRQ_DETECT_UNMASK     = GPIOC_BASE + 0x720;
 
        GPIOD_BASE  = 0x000dc000;
        GPIOD_DATA          = GPIOD_BASE + 0x000;
        GPIOD_DIR           = GPIOD_BASE + 0x400;
        GPIOD_IS            = GPIOD_BASE + 0x404;
        GPIOD_IBE           = GPIOD_BASE + 0x408;
        GPIOD_IEV           = GPIOD_BASE + 0x40c;
        GPIOD_IE            = GPIOD_BASE + 0x410;
        GPIOD_RIS           = GPIOD_BASE + 0x414;
        GPIOD_MIS           = GPIOD_BASE + 0x418;
        GPIOD_IC            = GPIOD_BASE + 0x41c;
        GPIOD_AFSEL         = GPIOD_BASE + 0x420;
        GPIOD_GPIOLOCK          = GPIOD_BASE + 0x520;
        GPIOD_GPIOCR            = GPIOD_BASE + 0x524;
        GPIOD_PMUX          = GPIOD_BASE + 0x700;
        GPIOD_P_EDGE_CTRL       = GPIOD_BASE + 0x704;
        GPIOD_PI_EN         = GPIOD_BASE + 0x710;
        GPIOD_IRQ_DETECT_ACK        = GPIOD_BASE + 0x718;
        GPIOD_USB_IRQ_ACK       = GPIOD_BASE + 0x71c;
        GPIOD_IRQ_DETECT_UNMASK     = GPIOD_BASE + 0x720;
 
        UDMA_BASE   = 0x000ff000;
        PKA_BASE    = 0x04004000;
        PKA_SRAM_BASE   = 0x04006000;
 
        CCTEST_BASE = 0x04010000;
 
        CCTEST_IO           = CCTEST_BASE + 0x000;
        CCTEST_OBSSEL           = CCTEST_BASE + 0x014;
        CCTEST_TR0          = CCTEST_BASE + 0x034;
        CCTEST_USBCTRL          = CCTEST_BASE + 0x050;
 
    } > PERIPHERAL
 
 
    /* Peripheral base addresses */
    .sysperiph : ALIGN(4)
    {
        /*
         * Addresses are relative to the start of
         * the SYSPERIPH block (0xe0000000)
         */
        SYSTICK_STCTRL          = 0x0000e010;
        SYSTICK_STRELOAD        = 0x0000e014;
        SYSTICK_STCURRENT       = 0x0000e018;
        SYSTICK_STCALIB         = 0x0000e01c;
 
        /*
         * There's a lot of registers, all seemingly arrays, so I'll just
         * put the start of each block here and we'll declare them arrays
         * in C.
         */
        NVIC_TYPE           = 0x0000e004;
        NVIC_EN             = 0x0000e100;   /*… 0xe11c */
        NVIC_DIS            = 0x0000e180;   /*… 0xe19c */
        NVIC_PEND           = 0x0000e200;   /*… 0xe21c */
        NVIC_UNPEND         = 0x0000e280;   /*… 0xe29c */
        NVIC_ACTIVE         = 0x0000e300;   /*… 0xe31c */
        NVIC_PRI            = 0x0000e400;   /*… 0xe41f */
        NVIC_SWTRIG         = 0x0000ef00;
 
        SCB_ACTLR           = 0x0000e008;
        SCB_CPUID           = 0x0000ed00;
        SCB_INTCTRL         = 0x0000ed04;
        SCB_VTABLE          = 0x0000ed08;
        SCB_APINT           = 0x0000ed0c;
        SCB_SYSCTRL         = 0x0000ed10;
        SCB_CFGCTRL         = 0x0000ed14;
        SCB_SYSPRI1         = 0x0000ed18;
        SCB_SYSPRI2         = 0x0000ed1c;
        SCB_SYSPRI3         = 0x0000ed20;
        SCB_SYSHNDCTRL          = 0x0000ed24;
        SCB_FAULTSTAT           = 0x0000ed28;
        SCB_HFAULTSTAT          = 0x0000ed2c;
        SCB_DFAULTSTAT          = 0x0000ed30;
        SCB_AFAULTSTAT          = 0x0000ed3c;
        SCB_MMADDR          = 0x0000ed34;
        SCB_FAULTADDR           = 0x0000ed38;
 
        MPU_TYPE            = 0x0000ed90;
        MPU_CTRL            = 0x0000ed94;
        MPU_NUMBER          = 0x0000ed98;
        MPU_REGION_BASE         = 0x0000ed9c;
        MPU_REGION_ATTR         = 0x0000eda0;
        MPU_REGION_BASE1        = 0x0000eda4;
        MPU_REGION_ATTR1        = 0x0000eda8;
        MPU_REGION_BASE2        = 0x0000edac;
        MPU_REGION_ATTR2        = 0x0000edb0;
        MPU_REGION_BASE3        = 0x0000edb4;
        MPU_REGION_ATTR3        = 0x0000edb8;
 
    } > SYSPERIPH
}
