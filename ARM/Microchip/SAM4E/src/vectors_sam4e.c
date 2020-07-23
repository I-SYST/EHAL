/**-------------------------------------------------------------------------
@file	Vector_SAM4E.c

@brief	Interrupt Vectors table for ARM Cortex-M4 specific to SAM4E
		 CMSIS & GCC compiler
		 linker section name .Vectors is used for the table

@author	Hoang Nguyen Hoan
@date	May 31, 2020

@licanse

Copyright (c) 2020, I-SYST inc., all rights reserved

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
#include "sam4e.h"
#if __FPU_USED /* CMSIS defined value to indicate usage of FPU */
//#include "fpu.h"
#endif

extern unsigned long __StackTop;
extern void ResetEntry(void);
extern void Reset_Handler(void);
extern char Image$$ER_ZI$$Base[];
extern char Image$$ARM_LIB_STACK$$ZI$$Base[];

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
__attribute__((weak, alias("DEF_IRQHandler"))) void SUPC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RSTC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RTC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RTT_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void WDT_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PMC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void EFC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UART0_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void Dummy_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PIOA_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PIOB_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PIOC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PIOD_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PIOE_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void USART0_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void USART1_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void HSMCI_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TWI0_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TWI1_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPI_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DMAC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TC0_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TC1_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TC2_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TC3_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TC4_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TC5_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TC6_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TC7_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TC8_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void AFEC0_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void AFEC1_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DACC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ACC_Handler(void);
#if (__FPU_USED == 1)
__WEAK void ARM_Handler(void);
#else
__attribute__((weak, alias("DEF_IRQHandler"))) void ARM_Handler(void);
#endif
__attribute__((weak, alias("DEF_IRQHandler"))) void UDP_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PWM_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CAN0_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CAN1_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void AES_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GMAC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UART1_Handler(void);

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
void (* const g_Vectors[100])(void) = {
#endif
#if defined ( __ARMCC_VERSION )
	(void (*)(void) )((uint32_t)0x20000000 + 0x10000),
	Reset_Handler,
#else
	(void (*)(void) )((uint32_t)&__StackTop),
	ResetEntry,
#endif
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

/* External Interrupts */
	SUPC_Handler,
	RSTC_Handler,
	RTC_Handler,
	RTT_Handler,
	WDT_Handler,
	PMC_Handler,
	EFC_Handler,
	UART0_Handler,
	Dummy_Handler,
	PIOA_Handler,
	PIOB_Handler,
	PIOC_Handler,
	PIOD_Handler,
	PIOE_Handler,
	USART0_Handler,
	USART1_Handler,
	HSMCI_Handler,
	TWI0_Handler,
	TWI1_Handler,
	SPI_Handler,
	DMAC_Handler,
	TC0_Handler,
	TC1_Handler,
	TC2_Handler,
	TC3_Handler,
	TC4_Handler,
	TC5_Handler,
	TC6_Handler,
	TC7_Handler,
	TC8_Handler,
	AFEC0_Handler,
	AFEC1_Handler,
	DACC_Handler,
	ACC_Handler,
	ARM_Handler,
	UDP_Handler,
	PWM_Handler,
	CAN0_Handler,
	CAN1_Handler,
	AES_Handler,
	Dummy_Handler,
	Dummy_Handler,
	Dummy_Handler,
	Dummy_Handler,
	GMAC_Handler,
	UART1_Handler
};

#if (__FPU_USED == 1)
// Function handles and clears exception flags in FPSCR register and at the stack.
// During interrupt, handler execution FPU registers might be copied to the stack
// (see lazy stacking option) and it is necessary to clear data at the stack
// which will be recovered in the return from interrupt handling.
__WEAK void ARM_Handler(void)
{
    // Prepare pointer to stack address with pushed FPSCR register (0x40 is FPSCR register offset in stacked data)
    uint32_t * fpscr = (uint32_t * )(FPU->FPCAR + 0x40);
    // Execute FPU instruction to activate lazy stacking
    (void)__get_FPSCR();
    // Clear flags in stacked FPSCR register. To clear IDC, IXC, UFC, OFC, DZC and IOC flags, use 0x0000009F mask.
    *fpscr = *fpscr & ~(0x0000009F);
}
#endif

