/**-------------------------------------------------------------------------
@file	Vector_STM32L401xx.c

@brief	Interrupt Vectors table for ARM Cortex-M4 STM32L401xx.

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
__attribute__((weak, alias("DEF_IRQHandler"))) void MemManage_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void BusFault_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UsageFault_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SVC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DebugMon_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PendSV_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SysTick_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void WWDG_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PVD_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TAMP_STAMP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RTC_WKUP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FLASH_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RCC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Stream0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Stream1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Stream2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Stream3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Stream4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Stream5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Stream6_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ADC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI9_5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIM1_BRK_TIM9_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIM1_UP_TIM10_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIM1_TRG_COM_TIM11_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIM1_CC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIM2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIM3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIM4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I2C1_EV_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I2C1_ER_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I2C2_EV_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I2C2_ER_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPI1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPI2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void USART1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void USART2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI15_10_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RTC_Alarm_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void OTG_FS_WKUP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Stream7_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SDIO_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIM5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPI3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2_Stream0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2_Stream1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2_Stream2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2_Stream3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2_Stream4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void OTG_FS_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2_Stream5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2_Stream6_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2_Stream7_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void USART6_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I2C3_EV_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I2C3_ER_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FPU_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPI4_IRQHandler(void);

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
	MemManage_Handler,
	BusFault_Handler,
	UsageFault_Handler,
	0, 0, 0, 0,
	SVC_Handler,
	DebugMon_Handler,
	0,
	PendSV_Handler,
	SysTick_Handler,

// STM32L011xx specific
    WWDG_IRQHandler,					// Window WatchDog
    PVD_IRQHandler,                  // PVD through EXTI Line detection
	TAMP_STAMP_IRQHandler,         	// Tamper and TimeStamps through the EXTI line
	RTC_WKUP_IRQHandler,             // RTC Wakeup through the EXTI line
    FLASH_IRQHandler,                // FLASH
    RCC_IRQHandler,                  // RCC
    EXTI0_IRQHandler,              	// EXTI Line 0
    EXTI1_IRQHandler,              	// EXTI Line 1
    EXTI2_IRQHandler,              	// EXTI Line 2
	EXTI3_IRQHandler,				// EXIT Line 3
    EXTI4_IRQHandler,             	// EXTI Line 4
	DMA1_Stream0_IRQHandler,         // DMA1 Stream 0
	DMA1_Stream1_IRQHandler,         // DMA1 Stream 1
	DMA1_Stream2_IRQHandler,         // DMA1 Stream 2
	DMA1_Stream3_IRQHandler,         // DMA1 Stream 3
	DMA1_Stream4_IRQHandler,         // DMA1 Stream 4
	DMA1_Stream5_IRQHandler,         // DMA1 Stream 5
	DMA1_Stream6_IRQHandler,         // DMA1 Stream 6
	ADC_IRQHandler,                  // ADC1
	0, 0, 0, 0,                      // Reserved
	EXTI9_5_IRQHandler,              // External Line[9:5]s
	TIM1_BRK_TIM9_IRQHandler,        // TIM1 Break and TIM9
	TIM1_UP_TIM10_IRQHandler,        // TIM1 Update and TIM10
	TIM1_TRG_COM_TIM11_IRQHandler,   // TIM1 Trigger and Commutation and TIM11
	TIM1_CC_IRQHandler,              // TIM1 Capture Compare
	TIM2_IRQHandler,                 // TIM2
	TIM3_IRQHandler,                 // TIM3
	TIM4_IRQHandler,                 // TIM4
	I2C1_EV_IRQHandler,              // I2C1 Event
	I2C1_ER_IRQHandler,              // I2C1 Error
	I2C2_EV_IRQHandler,              // I2C2 Event
	I2C2_ER_IRQHandler,              // I2C2 Error
	SPI1_IRQHandler,                 // SPI1
	SPI2_IRQHandler,                 // SPI2
	USART1_IRQHandler,               // USART1
	USART2_IRQHandler,               // USART2
	0,               				// Reserved
	EXTI15_10_IRQHandler,           	// External Line[15:1
	RTC_Alarm_IRQHandler,            // RTC Alarm (A and B) through EXTI Line
	OTG_FS_WKUP_IRQHandler,          // USB OTG FS Wakeup through EXTI lin/
	0, 0, 0, 0,                      // Reserved
	DMA1_Stream7_IRQHandler,         // DMA1 Stream7
	0,                               // Reserved
	SDIO_IRQHandler,                 // SDIO
	TIM5_IRQHandler,                 // TIM5
	SPI3_IRQHandler,                 // SPI3
	0, 0, 0, 0,                      // Reserved
	DMA2_Stream0_IRQHandler,         // DMA2 Stream 0
	DMA2_Stream1_IRQHandler,         // DMA2 Stream 1
	DMA2_Stream2_IRQHandler,         // DMA2 Stream 2
	DMA2_Stream3_IRQHandler,         // DMA2 Stream 3
	DMA2_Stream4_IRQHandler,         // DMA2 Stream 4
	0, 0, 0, 0, 0, 0,	          	// Reserved
	OTG_FS_IRQHandler,               // USB OTG FS
	DMA2_Stream5_IRQHandler,         // DMA2 Stream 5
	DMA2_Stream6_IRQHandler,         // DMA2 Stream 6
	DMA2_Stream7_IRQHandler,         // DMA2 Stream 7
	USART6_IRQHandler,               // USART6
	I2C3_EV_IRQHandler,              // I2C3 event
	I2C3_ER_IRQHandler,              // I2C3 error
	0,                               // Reserved
	0,                               // Reserved
	0,                               // Reserved
	0,                               // Reserved
	0,                               // Reserved
	0,                               // Reserved
	0,                               // Reserved
	FPU_IRQHandler,                  // FPU
	0,                               // Reserved
	0,                               // Reserved
	SPI4_IRQHandler,                 // SPI4
};

const uint32_t g_iVectorSize = sizeof(g_Vectors) + 4;



