/**-------------------------------------------------------------------------
@file	iopinctrl.h

@brief	General I/O pin control implementation specific

This file must be named iopinctrl.h no matter which target

This is STM32L4x implementation


@author	Hoang Nguyen Hoan
@date	June. 3, 2019

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

#ifndef __IOPINCTRL_H__
#define __IOPINCTRL_H__

#include <stdint.h>
#include "stm32l4xx.h"
#include "coredev/iopincfg.h"

/**
 * @brief	Set gpio pin direction
 *
 * Change pin direction only without changing any other settings
 * for fast switching between In & Out
 *
 * @Param 	PortNo	: Port number
 * @Param	PinNo  	: Pin number
 * @Param	Dir     : I/O direction
 */
static inline __attribute__((always_inline)) void IOPinSetDir(int PortNo, int PinNo, IOPINDIR Dir) {
	GPIO_TypeDef *reg = (GPIO_TypeDef *)(GPIOA_BASE + PortNo * 0x400);

	PinNo <<= 1;

	reg->MODER &= ~(GPIO_MODER_MODE0_Msk << PinNo);
	if (Dir == IOPINDIR_OUTPUT)
	{
		reg->MODER |= GPIO_MODER_MODER0_0 << PinNo;
	}
}

/**
 * @brief	Read pin state
 *
 * @Param 	PortNo	: Port number
 * @Param	PinNo  	: Pin number
 *
 * @return	Pin state 1 or 0
 */
static inline __attribute__((always_inline)) int IOPinRead(int PortNo, int PinNo) {
	GPIO_TypeDef *reg = (GPIO_TypeDef *)(GPIOA_BASE + PortNo * 0x400);

	return (reg->IDR >> PinNo) & 1;
}

/**
 * @brief	Set pin to high (1 logic)
 *
 * @Param 	PortNo	: Port number
 * @Param	PinNo  	: Pin number
 */
static inline __attribute__((always_inline)) void IOPinSet(int PortNo, int PinNo) {
	GPIO_TypeDef *reg = (GPIO_TypeDef *)(GPIOA_BASE + PortNo * 0x400);

	reg->BSRR = (1 << PinNo) & 0xFFFF;
}

/**
 * @brief	Set pin to low (0 logic)
 *
 * @Param 	PortNo	: Port number
 * @Param	PinNo  	: Pin number
 */
static inline __attribute__((always_inline)) void IOPinClear(int PortNo, int PinNo) {
	GPIO_TypeDef *reg = (GPIO_TypeDef *)(GPIOA_BASE + PortNo * 0x400);

	reg->BRR = (1 << PinNo) & 0xFFFF;
}

/**
 * @brief	Toggle pin state (invert pin state)
 *
 * @Param 	PortNo	: Port number
 * @Param	PinNo  	: Pin number
 */
static inline __attribute__((always_inline)) void IOPinToggle(int PortNo, int PinNo) {
	GPIO_TypeDef *reg = (GPIO_TypeDef *)(GPIOA_BASE + PortNo * 0x400);

	reg->ODR ^= (1 << PinNo) & 0xFFFF;
}

/**
 * @brief	Read all pins on port
 *
 * @Param 	PortNo	: Port number
 *
 * @return	Bit field pin states
 */
static inline __attribute__((always_inline)) uint32_t IOPinReadPort(int PortNo) {
	GPIO_TypeDef *reg = (GPIO_TypeDef *)(GPIOA_BASE + PortNo * 0x400);

	return reg->IDR & 0xFFFF;
}

/**
 * @brief	Write state to all pin on port
 *
 * @Param 	PortNo	: Port number
 * @Param	Data	: Bit field state of all pins on port
 */
static inline __attribute__((always_inline)) void IOPinWritePort(int PortNo, uint32_t Data) {
	GPIO_TypeDef *reg = (GPIO_TypeDef *)(GPIOA_BASE + PortNo * 0x400);

	reg->ODR = Data & 0xFFFF;
}


#endif // __IOPINCTRL_H__

