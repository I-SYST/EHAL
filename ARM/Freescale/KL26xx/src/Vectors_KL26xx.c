/*--------------------------------------------------------------------------
File   : Vector_KL26xx.c

Author : Hoang Nguyen Hoan          Mar. 14, 2014

Desc   : Interrupt Vectors table for Freescale MKL26Zxx ARM Cortex-M0+
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
__attribute__((weak, alias("DEF_IRQHandler"))) void NMI_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void HardFault_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SVC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PendSV_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SysTick_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMA3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void FTFA_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void LVD_LVW_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void LLW_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I2C0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I2C1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPI0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPI1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UART0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UART1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UART2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ADC0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CMP0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TPM0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TPM1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TPM2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RTC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RTC_Seconds_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PIT_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I2S0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void USB0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DAC0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TSI0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void MCG_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void LPTMR0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PORTA_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PORTC_PORTD_IRQHandler(void);

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
	(void (*) )0xefffdb09,		// Checksum value
	0, 0, 0,
	SVC_Handler,
	0,
	0,
	PendSV_Handler,
	SysTick_Handler,

// External Interrupts
    DMA0_IRQHandler,  	// DMA channel 0 transfer complete/error interrupt
    DMA1_IRQHandler,  	// DMA channel 1 transfer complete/error interrupt
    DMA2_IRQHandler,  	// DMA channel 2 transfer complete/error interrupt
    DMA3_IRQHandler,  	// DMA channel 3 transfer complete/error interrupt
    0,					// Reserved interrupt 20
    FTFA_IRQHandler,  	// FTFA command complete/read collision interrupt
    LVD_LVW_IRQHandler,	// Low Voltage Detect, Low Voltage Warning
    LLW_IRQHandler,		// Low Leakage Wakeup
    I2C0_IRQHandler,	// I2C0 interrupt
    I2C1_IRQHandler,	// I2C0 interrupt 25
    SPI0_IRQHandler,  	// SPI0 interrupt
    SPI1_IRQHandler,	// SPI1 interrupt
    UART0_IRQHandler,	// UART0 status/error interrupt
    UART1_IRQHandler,	// UART1 status/error interrupt
    UART2_IRQHandler,	// UART2 status/error interrupt
    ADC0_IRQHandler,	// ADC0 interrupt
    CMP0_IRQHandler,	// CMP0 interrupt
    TPM0_IRQHandler,	// TPM0 fault, overflow and channels interrupt
    TPM1_IRQHandler,	// TPM1 fault, overflow and channels interrupt
    TPM2_IRQHandler,	// TPM2 fault, overflow and channels interrupt
    RTC_IRQHandler,		// RTC interrupt
    RTC_Seconds_IRQHandler,	// RTC seconds interrupt
    PIT_IRQHandler,			// PIT timer interrupt
    I2S0_IRQHandler,		// I2S0 transmit interrupt
    USB0_IRQHandler,		// USB0 interrupt
	DAC0_IRQHandler,		// DAC0 interrupt
    TSI0_IRQHandler,		// TSI0 interrupt
    MCG_IRQHandler,			// MCG interrupt
	LPTMR0_IRQHandler,		// LPTimer interrupt
    0,						// Reserved interrupt 45
	PORTA_IRQHandler,		// Port A interrupt
	PORTC_PORTD_IRQHandler,		// Port D interrupt
};

const uint32_t g_iVectorSize = sizeof(g_Vectors) + 4;
