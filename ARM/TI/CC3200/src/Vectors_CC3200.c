/*--------------------------------------------------------------------------
File   : Vectors_CC3200.c

Author : Hoang Nguyen Hoan          Oct. 3, 2014

Desc   : Interrupt vectors for ARM Cortex-M4 TI CC3200 specific

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
#include <string.h>
#include <sys/types.h>

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

__attribute__((weak, alias("DEF_IRQHandler"))) void GPIOA_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIOB_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIOC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIOD_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UART0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UART1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ADC0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ADC1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ADC2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ADC3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void WDT_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER0A_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER0B_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER1A_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER1B_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER2A_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER2B_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FLASH_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER3A_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER3B_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMAErr_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SHA_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void AES_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DES_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SDHost_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I2S_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void Camera_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void NMP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PRC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SharedSPI_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPI_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void LinkSPI_IRQHandler(void);


/**
 * This interrupt vector is by default located in FLASH. Though it can not be
 * changed at runtime. All fcuntions in the vector are weak.  it can be
 * overloaded by application function
 *
 */
__attribute__ ((section(".intvect"), used))
void (* const g_Vectors[])(void) =
{
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

	// CC3200 specifics
	GPIOA_IRQHandler,
	GPIOB_IRQHandler,
	GPIOC_IRQHandler,
	GPIOD_IRQHandler,
	0,
	UART0_IRQHandler,
	UART1_IRQHandler,
	0, 0, 0, 0, 0,
	ADC0_IRQHandler,
	ADC1_IRQHandler,
	ADC2_IRQHandler,
	ADC3_IRQHandler,
	WDT_IRQHandler,
	TIMER0A_IRQHandler,
	TIMER0B_IRQHandler,
	TIMER1A_IRQHandler,
	TIMER1B_IRQHandler,
	TIMER2A_IRQHandler,
	TIMER2B_IRQHandler,
	0, 0, 0, 0,
	FLASH_IRQHandler,
	0, 0, 0, 0, 0,
	TIMER3A_IRQHandler,
	TIMER3B_IRQHandler,
    0, 0, 0, 0, 0, 0, 0, 0, 0,                      // Reserved
	DMA_IRQHandler,
	DMAErr_IRQHandler,
    0,0,0,0,0,0,0,0,0,0,                    // Reserved
    0,0,0,0,0,0,0,0,0,0,                    // Reserved
    0,0,0,0,0,0,0,0,0,0,                    // Reserved
    0,0,0,0,0,0,0,0,0,0,                    // Reserved
    0,0,0,0,0,0,0,0,0,0,                    // Reserved
    0,0,0,0,0,0,0,0,0,0,                    // Reserved
    0,0,0,0,0,0,0,0,0,0,                    // Reserved
    0,0,0,0,0,0,0,0,0,0,                    // Reserved
    0,0,0,0,0,0,0,0,0,0,                    // Reserved
    0,0,0,0,0,0,0,0,0,0,                    // Reserved
    SHA_IRQHandler,
    0, 0,
    AES_IRQHandler,
    0,
    DES_IRQHandler,
    0, 0, 0, 0, 0,
    SDHost_IRQHandler,
    0,
	I2S_IRQHandler,
	0,
	Camera_IRQHandler,
	0, 0, 0, 0, 0, 0, 0,
	NMP_IRQHandler,
	PRC_IRQHandler,
	0, 0,
	SharedSPI_IRQHandler,
	SPI_IRQHandler,
	LinkSPI_IRQHandler,
    0,0,0,0,0,0,0,0,0,0,                    // Reserved
    0,0,0,0,0,0,0,0,0,0,                    // Reserved
    0,0,0,0,0,0,0,0,0,0,                    // Reserved
    0,0,0,0,0,0,0,0,0,0,                    // Reserved
    0,0,0,0,0,0,0,0,0,0,                    // Reserved
    0,0,0,0,0,0,0,0,0,0,                    // Reserved
    0,0                                     // Reserved
};

