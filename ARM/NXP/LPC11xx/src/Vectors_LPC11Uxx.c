/*--------------------------------------------------------------------------
File   : Vector_M0.c

Author : Hoang Nguyen Hoan          Mar. 14, 2014

Desc   : Interrupt Vectors table for ARM Cortex-M0
		 CMSIS & GCC compiler
		 linker section name .Vectors is used for the table

Copyright (c) 2014, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/
#include <stdint.h>
extern unsigned long __StackTop;
extern void ResetEntry(void);

void DEF_IRQHandler(void) { while(1); }
__attribute__((weak, alias("DEF_IRQHandler"))) void NMI_Handler(void) { while(1);}
__attribute__((weak, alias("DEF_IRQHandler"))) void HardFault_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SVC_Handler(void) { while(1);}
__attribute__((weak, alias("DEF_IRQHandler"))) void PendSV_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SysTick_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FLEX_INT0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FLEX_INT1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FLEX_INT2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FLEX_INT3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FLEX_INT4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FLEX_INT5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FLEX_INT6_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FLEX_INT7_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GINT0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GINT1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SSP1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I2C_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER16_0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER16_1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER32_0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER32_1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SSP0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UART_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void USB_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void USB_FIQHandler(void) { }
__attribute__((weak, alias("DEF_IRQHandler"))) void ADC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void WDT_IRQHandler(void) { while(1);}
__attribute__((weak, alias("DEF_IRQHandler"))) void BOD_IRQHandler(void) { }
__attribute__((weak, alias("DEF_IRQHandler"))) void FMC_IRQHandler(void) { while(1);}
__attribute__((weak, alias("DEF_IRQHandler"))) void USBWakeup_IRQHandler(void) { }

/**
 * This interrupt vector is by default located in FLASH. Though it can not be
 * changed at runtime. All functions in the vector are weak.  it can be
 * overloaded by application function
 *
 */
__attribute__ ((section(".intvect"), used))
void (* const g_Vectors[])(void) =
{
	(void (*) )((int32_t)&__StackTop),
	ResetEntry,
	NMI_Handler,
	HardFault_Handler,
	0,
	0,
	0,
	(void (*) )0xefffd13b,		// Checksum value
	0, 0, 0,
	SVC_Handler,
	0,
	0,
	PendSV_Handler,
	SysTick_Handler,

/* External Interrupts */
    FLEX_INT0_IRQHandler,             //  0 - GPIO pin interrupt 0
    FLEX_INT1_IRQHandler,             //  1 - GPIO pin interrupt 1
    FLEX_INT2_IRQHandler,             //  2 - GPIO pin interrupt 2
    FLEX_INT3_IRQHandler,             //  3 - GPIO pin interrupt 3
    FLEX_INT4_IRQHandler,             //  4 - GPIO pin interrupt 4
    FLEX_INT5_IRQHandler,             //  5 - GPIO pin interrupt 5
    FLEX_INT6_IRQHandler,             //  6 - GPIO pin interrupt 6
    FLEX_INT7_IRQHandler,             //  7 - GPIO pin interrupt 7
    GINT0_IRQHandler,                 //  8 - GPIO GROUP0 interrupt
    GINT1_IRQHandler,                 //  9 - GPIO GROUP1 interrupt
    0,                                // 10 - Reserved
    0,                                // 11 - Reserved
    0,                                // 12 - Reserved
    0,                                // 13 - Reserved
    SSP1_IRQHandler,                  // 14 - SPI/SSP1 Interrupt
    I2C_IRQHandler,                   // 15 - I2C0
    TIMER16_0_IRQHandler,             // 16 - CT16B0 (16-bit Timer 0)
    TIMER16_1_IRQHandler,             // 17 - CT16B1 (16-bit Timer 1)
    TIMER32_0_IRQHandler,             // 18 - CT32B0 (32-bit Timer 0)
    TIMER32_1_IRQHandler,             // 19 - CT32B1 (32-bit Timer 1)
    SSP0_IRQHandler,                  // 20 - SPI/SSP0 Interrupt
    UART_IRQHandler,                  // 21 - UART0
    USB_IRQHandler,                   // 22 - USB IRQ
    USB_FIQHandler,                   // 23 - USB FIQ
    ADC_IRQHandler,                   // 24 - ADC (A/D Converter)
    WDT_IRQHandler,                   // 25 - WDT (Watchdog Timer)
    BOD_IRQHandler,                   // 26 - BOD (Brownout Detect)
    FMC_IRQHandler,                   // 27 - IP2111 Flash Memory Controller
    0,                                // 28 - Reserved
    0,                                // 29 - Reserved
    USBWakeup_IRQHandler,             // 30 - USB wake-up interrupt
    0,                                // 31 - Reserved
};

const uint32_t g_iVectorSize = sizeof(g_Vectors) + 4;
