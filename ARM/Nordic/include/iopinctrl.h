/*--------------------------------------------------------------------------
File   : iopinctrl.h

Author : Hoang Nguyen Hoan          June. 2, 2014

Desc   : General I/O pin control implementation specific 
		 This file must be named iopinctrl.h no matter which target

		 This is nRF51 implementation

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
#ifndef __IOPINCTRL_H__
#define __IOPINCTRL_H__

#include <stdint.h>
#include "nrf.h"
#include "iopincfg.h"

/**
 * @brief	Set gpio pin direction
 *
 * 	change pin direction only without changing any other settings
 * 	for fast switching between In & Out
 *
 * @Param 	PortNo	: Port number
 * 			PinNo  	: Pin number
 * 			Dir     : I/O direction
 */
static inline __attribute__((always_inline)) void IOPinSetDir(int PortNo, int PinNo, IOPINDIR Dir) {
	NRF_GPIO_Type *reg = NRF_GPIO;

#ifdef NRF52840_XXAA
	if (PortNo == 1)
	{
		reg = NRF_P1;
	}

#endif

	if (Dir == IOPINDIR_OUTPUT)
		reg->DIRSET = (1 << PinNo);
	else if (Dir == IOPINDIR_INPUT)
		reg->DIRCLR = (1 << PinNo);
}

/**
 * @brief	Read pin state
 *
 * @Param 	PortNo	: Port number
 * 			PinNo  	: Pin number
 *
 * @return	Pin state 1 or 0
 */
static inline __attribute__((always_inline)) int IOPinRead(int PortNo, int PinNo) {
	NRF_GPIO_Type *reg = NRF_GPIO;

#ifdef NRF52840_XXAA
	if (PortNo == 1)
	{
		reg = NRF_P1;
	}

#endif

	return (reg->IN >> PinNo) & 1;
}

/**
 * @brief	Set pin to high (1 logic)
 *
 * @Param 	PortNo	: Port number
 * 			PinNo  	: Pin number
 */
static inline __attribute__((always_inline)) void IOPinSet(int PortNo, int PinNo) {
	NRF_GPIO_Type *reg = NRF_GPIO;

#ifdef NRF52840_XXAA
	if (PortNo == 1)
	{
		reg = NRF_P1;
	}

#endif

	reg->OUTSET = (1 << PinNo);
}

/**
 * @brief	Set pin to low (0 logic)
 *
 * @Param 	PortNo	: Port number
 * 			PinNo  	: Pin number
 */
static inline __attribute__((always_inline)) void IOPinClear(int PortNo, int PinNo) {
	NRF_GPIO_Type *reg = NRF_GPIO;

#ifdef NRF52840_XXAA
	if (PortNo == 1)
	{
		reg = NRF_P1;
	}

#endif

	reg->OUTCLR = (1 << PinNo);
}

/**
 * @brief	Toggle pin state (invert pin state)
 *
 * @Param 	PortNo	: Port number
 * 			PinNo  	: Pin number
 */
static inline __attribute__((always_inline)) void IOPinToggle(int PortNo, int PinNo) {
	NRF_GPIO_Type *reg = NRF_GPIO;

#ifdef NRF52840_XXAA
	if (PortNo == 1)
	{
		reg = NRF_P1;
	}

#endif

	reg->OUT = NRF_GPIO->OUT ^ (1 << PinNo);
}

/**
 * @brief	Read all pins on port
 *
 * @Param 	PortNo	: Port number
 *
 * @return	Bit field pin states
 */
static inline __attribute__((always_inline)) uint32_t IOPinReadPort(int PortNo) {
	NRF_GPIO_Type *reg = NRF_GPIO;

#ifdef NRF52840_XXAA
	if (PortNo == 1)
	{
		reg = NRF_P1;
	}

#endif

	return reg->IN;
}

/**
 * @brief	Write state to all pin on port
 *
 * @Param 	PortNo	: Port number
 * 			Data	: Bit field state of all pins on port
 */
static inline __attribute__((always_inline)) void IOPinWritePort(int PortNo, uint32_t Data) {
	NRF_GPIO_Type *reg = NRF_GPIO;

#ifdef NRF52840_XXAA
	if (PortNo == 1)
	{
		reg = NRF_P1;
	}

#endif

	reg->OUT = Data;
}

#endif	// __IOPINCTRL_H__
