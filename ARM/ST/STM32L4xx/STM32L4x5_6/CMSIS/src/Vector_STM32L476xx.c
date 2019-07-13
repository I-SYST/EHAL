/**-------------------------------------------------------------------------
@file	Vector_STM32L476xx.c

@brief	Interrupt Vectors table for ARM Cortex-M4 STM32L476xx.

		 CMSIS & GCC compiler
		 linker section name .Vectors is used for the table

@author	Hoang Nguyen Hoan
@date	Jul. 1, 2019

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

extern unsigned long __StackTop;
extern void ResetEntry(void);
extern unsigned long __ICFEDIT_region_RAM_end__;

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
__attribute__((weak, alias("DEF_IRQHandler"))) void PVD_PVM_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TAMP_STAMP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RTC_WKUP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FLASH_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RCC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Channel1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Channel2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Channel3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Channel4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Channel5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Channel6_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_Channel7_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ADC1_2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CAN1_TX_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CAN1_RX0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CAN1_RX1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CAN1_SCE_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI9_5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIM1_BRK_TIM15_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIM1_UP_TIM16_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIM1_TRG_COM_TIM17_IRQHandler(void);
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
__attribute__((weak, alias("DEF_IRQHandler"))) void USART3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EXTI15_10_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RTC_Alarm_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DFSDM1_FLT3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIM8_BRK_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIM8_UP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIM8_TRG_COM_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIM8_CC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ADC3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FMC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SDMMC1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIM5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPI3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UART4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UART5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIM6_DAC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIM7_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2_Channel1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2_Channel2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2_Channel3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2_Channel4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2_Channel5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DFSDM1_FLT0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DFSDM1_FLT1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DFSDM1_FLT2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void COMP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void LPTIM1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void LPTIM2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void OTG_FS_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2_Channel6_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2_Channel7_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void LPUART1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void QUADSPI_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I2C3_EV_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I2C3_ER_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SAI1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SAI2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWPMI1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TSC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void LCD_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RNG_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FPU_IRQHandler(void);

/**
 * This interrupt vector is by default located in FLASH. Though it can not be
 * changed at runtime. All functions in the vector are weak.  it can be
 * overloaded by application function
 *
 */
#ifdef __ICCARM__
__attribute__ ((section(".intvec"), used))
void (* const __vector_table[])(void) = {
#else
__attribute__ ((section(".intvect"), used))
void (* const g_Vectors[])(void) = {
#endif
#ifdef __ICCARM__
  (void (*)(void) )((uint32_t)&__ICFEDIT_region_RAM_end__),
#else
  	(void (*)(void) )((uint32_t)&__StackTop),
#endif
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
	PVD_PVM_IRQHandler,
	TAMP_STAMP_IRQHandler,
	RTC_WKUP_IRQHandler,
	FLASH_IRQHandler,
	RCC_IRQHandler,
	EXTI0_IRQHandler,
	EXTI1_IRQHandler,
	EXTI2_IRQHandler,
	EXTI3_IRQHandler,
	EXTI4_IRQHandler,
	DMA1_Channel1_IRQHandler,
	DMA1_Channel2_IRQHandler,
	DMA1_Channel3_IRQHandler,
	DMA1_Channel4_IRQHandler,
	DMA1_Channel5_IRQHandler,
	DMA1_Channel6_IRQHandler,
	DMA1_Channel7_IRQHandler,
	ADC1_2_IRQHandler,
	CAN1_TX_IRQHandler,
	CAN1_RX0_IRQHandler,
	CAN1_RX1_IRQHandler,
	CAN1_SCE_IRQHandler,
	EXTI9_5_IRQHandler,
	TIM1_BRK_TIM15_IRQHandler,
	TIM1_UP_TIM16_IRQHandler,
	TIM1_TRG_COM_TIM17_IRQHandler,
	TIM1_CC_IRQHandler,
	TIM2_IRQHandler,
	TIM3_IRQHandler,
	TIM4_IRQHandler,
	I2C1_EV_IRQHandler,
	I2C1_ER_IRQHandler,
	I2C2_EV_IRQHandler,
	I2C2_ER_IRQHandler,
	SPI1_IRQHandler,
	SPI2_IRQHandler,
	USART1_IRQHandler,
	USART2_IRQHandler,
	USART3_IRQHandler,
	EXTI15_10_IRQHandler,
	RTC_Alarm_IRQHandler,
	DFSDM1_FLT3_IRQHandler,
	TIM8_BRK_IRQHandler,
	TIM8_UP_IRQHandler,
	TIM8_TRG_COM_IRQHandler,
	TIM8_CC_IRQHandler,
	ADC3_IRQHandler,
	FMC_IRQHandler,
	SDMMC1_IRQHandler,
	TIM5_IRQHandler,
	SPI3_IRQHandler,
	UART4_IRQHandler,
	UART5_IRQHandler,
	TIM6_DAC_IRQHandler,
	TIM7_IRQHandler,
	DMA2_Channel1_IRQHandler,
	DMA2_Channel2_IRQHandler,
	DMA2_Channel3_IRQHandler,
	DMA2_Channel4_IRQHandler,
	DMA2_Channel5_IRQHandler,
	DFSDM1_FLT0_IRQHandler,
	DFSDM1_FLT1_IRQHandler,
	DFSDM1_FLT2_IRQHandler,
	COMP_IRQHandler,
	LPTIM1_IRQHandler,
	LPTIM2_IRQHandler,
	OTG_FS_IRQHandler,
	DMA2_Channel6_IRQHandler,
	DMA2_Channel7_IRQHandler,
	LPUART1_IRQHandler,
	QUADSPI_IRQHandler,
	I2C3_EV_IRQHandler,
	I2C3_ER_IRQHandler,
	SAI1_IRQHandler,
	SAI2_IRQHandler,
	SWPMI1_IRQHandler,
	TSC_IRQHandler,
	LCD_IRQHandler,
	0,
	RNG_IRQHandler,
	FPU_IRQHandler
};
#ifdef __ICCARM__
const uint32_t g_iVectorSize = sizeof(__vector_table) + 4;
#else
const uint32_t g_iVectorSize = sizeof(g_Vectors) + 4;
#endif

