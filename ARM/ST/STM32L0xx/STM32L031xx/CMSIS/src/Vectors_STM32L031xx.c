/**-------------------------------------------------------------------------
@file	Vector_STM32L011xx.c

@brief	Interrupt Vectors table for ARM Cortex-M0 STM32L011xx.

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
__attribute__((weak, alias("DEF_IRQHandler"))) void PVD_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RTC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FLASH_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RCC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI0_1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI2_3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI4_15_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Channel1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Channel2_3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Channel4_5_6_7_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ADC1_COMP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void LPTIM1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIM2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIM21_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIM22_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I2C1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPI1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void USART2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void LPUART1_IRQHandler(void);

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
	0, 0, 0, 0, 0, 0, 0,
	SVC_Handler,
	0, 0,
	PendSV_Handler,
	SysTick_Handler,

// STM32L011xx specific
    WWDG_IRQHandler,					// Window WatchDog
    PVD_IRQHandler,                  // PVD through EXTI Line detection
    RTC_IRQHandler,                  // RTC through the EXTI line
    FLASH_IRQHandler,                // FLASH
    RCC_IRQHandler,                  // RCC
    EXTI0_1_IRQHandler,              // EXTI Line 0 and 1
    EXTI2_3_IRQHandler,              // EXTI Line 2 and 3
    EXTI4_15_IRQHandler,             // EXTI Line 4 to 15
	0,
	DMA1_Channel1_IRQHandler,        // DMA1 Channel 1
    DMA1_Channel2_3_IRQHandler,      // DMA1 Channel 2 and Channel 3
    DMA1_Channel4_5_6_7_IRQHandler,  // DMA1 Channel 4, Channel 5, Channel 6 and Channel 7
    ADC1_COMP_IRQHandler,            // ADC1, COMP1 and COMP2
    LPTIM1_IRQHandler,               // LPTIM1
	0,
    TIM2_IRQHandler,                 // TIM2
	0,
	0,
	0,
	0,                               // Reserved
    TIM21_IRQHandler,                // TIM21
	0,
	TIM22_IRQHandler,				// TIM22
    I2C1_IRQHandler,                 // I2C1
	0,
    SPI1_IRQHandler,                 // SPI1
	0,
	0,
	USART2_IRQHandler,               // USART2
    LPUART1_IRQHandler,              // LPUART1
	0,
	0
};

const uint32_t g_iVectorSize = sizeof(g_Vectors) + 4;



