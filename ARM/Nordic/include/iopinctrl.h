/**-------------------------------------------------------------------------
@file	iopinctrl.h

@brief	General I/O pin control implementation specific

This file must be named iopinctrl.h no matter which target

This is nRF5x implementation


@author	Hoang Nguyen Hoan
@date	June. 2, 2014

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
#ifndef __IOPINCTRL_H__
#define __IOPINCTRL_H__

#include <stdint.h>

#include "nrf.h"

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
#if defined(NRF91_SERIES) || defined(NRF53_SERIES)
#ifdef NRF5340_XXAA_NETWORK
	NRF_GPIO_Type *reg = NRF_P0_NS;
#else
	NRF_GPIO_Type *reg = NRF_P0_S;
	if (PortNo & 0x80)
	{
		reg = NRF_P0_NS;
	}
#endif
#else
	NRF_GPIO_Type *reg = NRF_GPIO;
#endif

#ifdef NRF52840_XXAA
	if (PortNo == 1)
	{
		reg = NRF_P1;
	}

#endif

	reg->PIN_CNF[PinNo] &= ~GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos;
	if (Dir == IOPINDIR_OUTPUT)
	{
		reg->PIN_CNF[PinNo] |= GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos;
		reg->DIRSET = (1 << PinNo);
	}
	else if (Dir == IOPINDIR_INPUT)
	{
		reg->DIRCLR = (1 << PinNo);
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
#if defined(NRF91_SERIES) || defined(NRF53_SERIES)
#ifdef NRF5340_XXAA_NETWORK
	return (NRF_P0_NS->IN >> PinNo) & 1;
#else
	if (PortNo & 0x80)
	{
		return (NRF_P0_NS->IN >> PinNo) & 1;
	}
	return (NRF_P0_S->IN >> PinNo) & 1;
#endif
#else

#ifdef NRF52840_XXAA
	if (PortNo == 1)
	{
		return (NRF_P1->IN >> PinNo) & 1;
	}
	else
#endif
	{
		return (NRF_GPIO->IN >> PinNo) & 1;
	}
#endif
}

/**
 * @brief	Set pin to high (1 logic)
 *
 * @Param 	PortNo	: Port number
 * @Param	PinNo  	: Pin number
 */
static inline __attribute__((always_inline)) void IOPinSet(int PortNo, int PinNo) {
#if defined(NRF91_SERIES) || defined(NRF53_SERIES)
#ifdef NRF5340_XXAA_NETWORK
	NRF_P0_NS->OUTSET = (1 << PinNo);
#else
	if (PortNo & 0x80)
	{
		NRF_P0_NS->OUTSET = (1 << PinNo);
	}
	else
	{
		NRF_P0_S->OUTSET = (1 << PinNo);
	}
#endif
#else

#ifdef NRF52840_XXAA
	if (PortNo == 1)
	{
		NRF_P1->OUTSET = (1 << PinNo);
	}
	else
#endif
	{
		NRF_GPIO->OUTSET = (1 << PinNo);
	}
#endif
}

/**
 * @brief	Set pin to low (0 logic)
 *
 * @Param 	PortNo	: Port number
 * @Param	PinNo  	: Pin number
 */
static inline __attribute__((always_inline)) void IOPinClear(int PortNo, int PinNo) {
#if defined(NRF91_SERIES) || defined(NRF53_SERIES)
#ifdef NRF5340_XXAA_NETWORK
	NRF_P0_NS->OUTCLR = (1 << PinNo);
#else
	if (PortNo & 0x80)
	{
		NRF_P0_NS->OUTCLR = (1 << PinNo);
	}
	else
	{
		NRF_P0_S->OUTCLR = (1 << PinNo);
	}
#endif
#else

#ifdef NRF52840_XXAA
	if (PortNo == 1)
	{
		NRF_P1->OUTCLR = (1 << PinNo);
	}
	else
#endif
	{
		NRF_GPIO->OUTCLR = (1 << PinNo);
	}
#endif
}

/**
 * @brief	Toggle pin state (invert pin state)
 *
 * @Param 	PortNo	: Port number
 * @Param	PinNo  	: Pin number
 */
static inline __attribute__((always_inline)) void IOPinToggle(int PortNo, int PinNo) {
#if defined(NRF91_SERIES) || defined(NRF53_SERIES)
#ifdef NRF5340_XXAA_NETWORK
	NRF_P0_NS->OUT ^= (1 << PinNo);
#else
	if (PortNo & 0x80)
	{
		NRF_P0_NS->OUT ^= (1 << PinNo);
	}
	else
	{
		NRF_P0_S->OUT ^= (1 << PinNo);
	}
#endif
#else

#ifdef NRF52840_XXAA
	if (PortNo == 1)
	{
		NRF_P1->OUT ^= (1 << PinNo);
	}
	else
#endif
	{
		NRF_GPIO->OUT ^= (1 << PinNo);
	}
#endif
}

/**
 * @brief	Read all pins on port
 *
 * @Param 	PortNo	: Port number
 *
 * @return	Bit field pin states
 */
static inline __attribute__((always_inline)) uint32_t IOPinReadPort(int PortNo) {
#if defined(NRF91_SERIES) || defined(NRF53_SERIES)
#ifdef NRF5340_XXAA_NETWORK
	return NRF_P0_NS->IN;
#else
	if (PortNo & 0x80)
	{
		return NRF_P0_NS->IN;
	}
	return NRF_P0_S->IN;
#endif
#else

#ifdef NRF52840_XXAA
	if (PortNo == 1)
	{
		return NRF_P1->IN;
	}
	else
#endif
	{
		return NRF_GPIO->IN;
	}
#endif
}

/**
 * @brief	Write state to all pin on port
 *
 * @Param 	PortNo	: Port number
 * @Param	Data	: Bit field state of all pins on port
 */
static inline __attribute__((always_inline)) void IOPinWritePort(int PortNo, uint32_t Data) {
#if defined(NRF91_SERIES) || defined(NRF53_SERIES)
#ifdef NRF5340_XXAA_NETWORK
	NRF_P0_NS->OUT = Data;
#else
	if (PortNo & 0x80)
	{
		NRF_P0_NS->OUT = Data;
	}
	else
	{
		NRF_P0_S->OUT = Data;
	}
#endif
#else

#ifdef NRF52840_XXAA
	if (PortNo == 1)
	{
		NRF_P1->OUT = Data;
	}
	else
#endif
	{
		NRF_GPIO->OUT = Data;
	}
#endif
}

#endif	// __IOPINCTRL_H__
