/*--------------------------------------------------------------------------
File   : Vector_nRF52.c

Author : Hoang Nguyen Hoan          July 6, 2015

Desc   : Interrupt Vectors table for ARM Cortex-M4 specific nRF52
		 CMSIS & GCC compiler
		 linker section name .Vectors is used for the table

Copyright (c) 2015, I-SYST inc., all rights reserved

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
__attribute__((weak, alias("DEF_IRQHandler"))) void NMI_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void HardFault_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SVC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PendSV_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SysTick_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void POWER_CLOCK_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RADIO_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UARTE0_UART0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void NFCT_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIOTE_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SAADC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RTC0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TEMP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RNG_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ECB_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CCM_AAR_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void WDT_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RTC1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void QDEC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void COMP_LPCOMP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI0_EGU0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI1_EGU1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI2_EGU2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI3_EGU3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI4_EGU4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI5_EGU5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PWM0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PDM_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void MWU_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PWM1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PWM2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPIM2_SPIS2_SPI2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RTC2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I2S_IRQHandler(void);

/**
 * This interrupt vector is by default located in FLASH. Though it can not be
 * changed at runtime. All fcuntions in the vector are weak.  it can be
 * overloaded by application function
 *
 */
__attribute__ ((section(".intvect"), used))
void (* const g_Vectors[])(void) =
{
	/*(void (*) )((int32_t)&__StackTop), This stack pointer address is hnadled in ld script*/
	ResetEntry,
	NMI_Handler,
	HardFault_Handler,
	0,
	0,
	0,
	0, 0, 0, 0,
	SVC_Handler,
	0,
	0,
	PendSV_Handler,
	SysTick_Handler,

/* External Interrupts */
    POWER_CLOCK_IRQHandler,
    RADIO_IRQHandler,
	UARTE0_UART0_IRQHandler,
	SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler,
	SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler,
	NFCT_IRQHandler,
    GPIOTE_IRQHandler,
	SAADC_IRQHandler,
    TIMER0_IRQHandler,
    TIMER1_IRQHandler,
    TIMER2_IRQHandler,
    RTC0_IRQHandler,
    TEMP_IRQHandler,
    RNG_IRQHandler,
    ECB_IRQHandler,
    CCM_AAR_IRQHandler,
    WDT_IRQHandler,
    RTC1_IRQHandler,
    QDEC_IRQHandler,
	COMP_LPCOMP_IRQHandler,
	SWI0_EGU0_IRQHandler,
	SWI1_EGU1_IRQHandler,
	SWI2_EGU2_IRQHandler,
	SWI3_EGU3_IRQHandler,
	SWI4_EGU4_IRQHandler,
	SWI5_EGU5_IRQHandler,
	TIMER3_IRQHandler,
	TIMER4_IRQHandler,
	PWM0_IRQHandler,
	PDM_IRQHandler,
    0,
    0,
	MWU_IRQHandler,
	PWM1_IRQHandler,
	PWM2_IRQHandler,
	SPIM2_SPIS2_SPI2_IRQHandler,
	RTC2_IRQHandler,
	I2S_IRQHandler,
	0,
	0
};

