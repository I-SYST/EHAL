/**-------------------------------------------------------------------------
@file	Vector_STM32F030x8.c

@brief	Interrupt Vectors table for ARM Cortex-M3 STM32F030x8.

		 CMSIS & GCC compiler
		 linker section name .Vectors is used for the table

@author	Hoang Nguyen Hoan
@date	Feb. 2, 2018

@license

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
__attribute__((weak, alias("DEF_IRQHandler"))) void WWDG_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RTC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FLASH_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RCC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI0_1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI2_3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI4_15_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Channel1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Channel2_3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Channel4_5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ADC1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIM1_BRK_UP_TRG_COM_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIM1_CC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIM3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIM6_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIM14_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIM15_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIM16_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIM17_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I2C1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I2C2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPI1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPI2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void USART1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void USART2_IRQHandler(void);

/**
 * This interrupt vector is by default located in FLASH. Though it can not be
 * changed at runtime. All functions in the vector are weak.  it can be
 * overloaded by application function
 *
 */
__attribute__ ((section(".intvect"), used))
void (* const g_Vectors[])(void) = {
	(void (*) )((int32_t)&__StackTop),
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

// STM32F030xC specific
    WWDG_IRQHandler,				// Window WatchDog
    0,
	RTC_IRQHandler,					// RTC through the EXTI line
    FLASH_IRQHandler,               // FLASH
    RCC_IRQHandler,                 // RCC
    EXTI0_1_IRQHandler,             // EXTI Line 0 and 1
    EXTI2_3_IRQHandler,             // EXTI Line 2 and 3
    EXTI4_15_IRQHandler,            // EXTI Line 4 to 15
	0,
	DMA1_Channel1_IRQHandler,       // DMA1 channel 1
	DMA1_Channel2_3_IRQHandler,     // DMA1 channel 2 and 3
	DMA1_Channel4_5_IRQHandler,     // DMA1 channel 4 and 5
	ADC1_IRQHandler,                // ADC1
	TIM1_BRK_UP_TRG_COM_IRQHandler,	// TIM1 Break, Update, Trigger and Commutation
	TIM1_CC_IRQHandler,				// TIM1 Capture Compare
	0,
	TIM3_IRQHandler,                // TIM3
	TIM6_IRQHandler,                // TIM6
	0,
	TIM14_IRQHandler,               // TIM14
	TIM15_IRQHandler,               // TIM15
	TIM16_IRQHandler,               // TIM16
	TIM17_IRQHandler,               // TIM17
	I2C1_IRQHandler,              	// I2C1 Event
	I2C2_IRQHandler,              	// I2C2 Event
	SPI1_IRQHandler,                // SPI1
	SPI2_IRQHandler,                // SPI2
	USART1_IRQHandler,              // USART1
	USART2_IRQHandler,              // USART2
	0,
	0,
	0
};

const uint32_t g_iVectorSize = sizeof(g_Vectors) + 4;



