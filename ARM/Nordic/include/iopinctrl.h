/*--------------------------------------------------------------------------
File   : iopinctrl.h

Author : Hoang Nguyen Hoan          June. 2, 2014

Desc   : General I/O pin control implementation specific 
		 This file must be named iopinctrl.h no matter which target

		 This is nRF5x implementation

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
#ifdef __plusplus
extern "C"
#endif
#include "nrf_gpio.h"

static inline void IOPinSetDir(int PortNo, int PinNo, IOPINDIR Dir)
{
	if (Dir == IOPINDIR_OUTPUT)
		nrf_gpio_pin_dir_set(PinNo, NRF_GPIO_PIN_DIR_OUTPUT);
	else if (Dir == IOPINDIR_INPUT)
		nrf_gpio_pin_dir_set(PinNo, NRF_GPIO_PIN_DIR_INPUT);
}

static inline int IOPinRead(int PortNo, int PinNo)
{
	return nrf_gpio_pin_read(PinNo);
}

static inline void IOPinSet(int PortNo, int PinNo)
{
	nrf_gpio_pin_set(PinNo);
}

static inline void IOPinClear(int PortNo, int PinNo)
{
	nrf_gpio_pin_clear(PinNo);
}

static inline void IOPinToggle(int PortNo, int PinNo)
{
	nrf_gpio_pin_toggle(PinNo);
}

static inline uint32_t IOPinReadPort(int PortNo)
{
	return nrf_gpio_pins_read();
}

static inline void IOPinWrite8Port(int PortNo, uint8_t Data)
{
	nrf_gpio_port_write((nrf_gpio_port_select_t)(NRF_GPIO_PORT_SELECT_PORT0 + PortNo), Data);
}

static inline void IOPinWrite16Port(int PortNo, uint16_t Data)
{
	nrf_gpio_port_write((nrf_gpio_port_select_t)(NRF_GPIO_PORT_SELECT_PORT0 + PortNo), Data & 0xFF);
	nrf_gpio_port_write((nrf_gpio_port_select_t)(NRF_GPIO_PORT_SELECT_PORT0 + PortNo + 1), Data >> 8);
}

static inline void IOPinWrite32Port(int PortNo, uint32_t Data)
{
	nrf_gpio_port_write((nrf_gpio_port_select_t)(NRF_GPIO_PORT_SELECT_PORT0 + PortNo), Data & 0xFF);
	nrf_gpio_port_write((nrf_gpio_port_select_t)(NRF_GPIO_PORT_SELECT_PORT0 + PortNo + 1), (Data >> 8) & 0xFF);
	nrf_gpio_port_write((nrf_gpio_port_select_t)(NRF_GPIO_PORT_SELECT_PORT0 + PortNo + 2), (Data >> 16) & 0xFF);
	nrf_gpio_port_write((nrf_gpio_port_select_t)(NRF_GPIO_PORT_SELECT_PORT0 + PortNo + 3), (Data >> 24) & 0xFF);
}

#endif	// __IOPINCTRL_H__
