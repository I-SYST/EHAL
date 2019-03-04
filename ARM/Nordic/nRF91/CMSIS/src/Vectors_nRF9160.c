/**-------------------------------------------------------------------------
@file	Vector_nRF9160.c

@brief	Interrupt Vectors table for ARM Cortex-M33 specific to nRF9160.

CMSIS & GCC compiler
linker section name .Vectors is used for the table


@author	Hoang Nguyen Hoan
@date	Feb. 26, 2019

@license

Copyright (c) 2019, I-SYST inc., all rights reserved

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

----------------------------------------------------------------------------*/
#include <stdint.h>
#include "nrf.h"

extern unsigned long __StackTop;
extern void ResetEntry(void);

void DEF_IRQHandler(void) { while(1); }
__attribute__((weak, alias("DEF_IRQHandler"))) void NMI_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void HardFault_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void MemoryManagement_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void BusFault_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UsageFault_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SecureFault_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SVC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DebugMon_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PendSV_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SysTick_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPU_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CLOCK_POWER_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UARTE2_SPIM2_SPIS2_TWIM2_TWIS2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UARTE3_SPIM3_SPIS3_TWIM3_TWIS3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIOTE0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SAADC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RTC0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RTC1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void WDT_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EGU0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EGU1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EGU2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EGU3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EGU4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EGU5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PWM0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PWM1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PWM2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PWM3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PDM_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I2S_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void IPC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIOTE1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void KMU_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CRYPTOCELL_IRQHandler(void);

#if (__FPU_USED == 1)
__WEAK void FPU_IRQHandler(void);
#else
__attribute__((weak, alias("DEF_IRQHandler"))) void FPU_IRQHandler(void);
#endif

/**
 * This interrupt vector is by default located in FLASH. Though it can not be
 * changed at runtime. All functions in the vector are weak.  it can be
 * overloaded by application function
 *
 */
__attribute__ ((section(".intvect"), used))
void (* const g_Vectors[200])(void) =
{
	(void (*) )((int32_t)&__StackTop),
	ResetEntry,
	NMI_Handler,
	HardFault_Handler,
	MemoryManagement_Handler,
	BusFault_Handler,
	UsageFault_Handler,
	SecureFault_Handler,
	0, 0, 0,
	SVC_Handler,
	DebugMon_Handler,
	0,
	PendSV_Handler,
	SysTick_Handler,

/* External Interrupts */
	0, 0, 0,
	SPU_IRQHandler,
	CLOCK_POWER_IRQHandler,
	0, 0,
	UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_IRQHandler,
	UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_IRQHandler,
	UARTE2_SPIM2_SPIS2_TWIM2_TWIS2_IRQHandler,
	UARTE3_SPIM3_SPIS3_TWIM3_TWIS3_IRQHandler,
	0,
	GPIOTE0_IRQHandler,
	SAADC_IRQHandler,
	SAADC_IRQHandler,
	TIMER0_IRQHandler,
    TIMER1_IRQHandler,
    TIMER2_IRQHandler,
	0, 0,
    RTC0_IRQHandler,
	RTC1_IRQHandler,
	0, 0,
	WDT_IRQHandler,
	0, 0,
	EGU0_IRQHandler,
	EGU1_IRQHandler,
	EGU2_IRQHandler,
	EGU3_IRQHandler,
	EGU4_IRQHandler,
	EGU5_IRQHandler,
	PWM0_IRQHandler,
	PWM1_IRQHandler,
	PWM2_IRQHandler,
	PWM3_IRQHandler,
	0,
	PDM_IRQHandler,
	0,
	I2S_IRQHandler,
	0,
	IPC_IRQHandler,
	0,
	FPU_IRQHandler,
	0, 0, 0, 0,
	GPIOTE1_IRQHandler,
	0, 0, 0, 0, 0, 0, 0,
	KMU_IRQHandler,
	0, 0, 0, 0, 0, 0,
	CRYPTOCELL_IRQHandler,
	0
};

