/**-------------------------------------------------------------------------
@file	iopincfg.h

@brief	Generic I/O pin configuration

This file contains only generic I/O pin configuration.

The I/O pin control is to be implemented per MCU device in iopinctrl.h

@author	Hoang Nguyen Hoan
@date	Nov. 20, 2011

@license

Copyright (c) 2011, I-SYST inc., all rights reserved

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
#ifndef __IOPINCFG_H__
#define __IOPINCFG_H__

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

/// I/O pin resistor configuration
typedef enum __iopin_resistor {
	IOPINRES_NONE,				//!< No pullup or pulldown
	IOPINRES_PULLUP,			//!< Pullup resistor
	IOPINRES_PULLDOWN,			//!< Pulldown resistor
	IOPINRES_FOLLOW				//!< Few MCUs support this mode
} IOPINRES;

/// I/O pin direction configuration
typedef enum __iopin_dir {
    IOPINDIR_INPUT  = 0,		//!< I/O pin as input
    IOPINDIR_OUTPUT = 1,		//!< I/O pin as output
    IOPINDIR_BI     = 2,		//!< Bidirectional, few MCUs support this mode
} IOPINDIR;

/// I/O pin type
typedef enum __iopin_type {
	IOPINTYPE_NORMAL    = 0,	//!< I/O pin normal type
	IOPINTYPE_OPENDRAIN = 1		//!< I/O pin open drain type
} IOPINTYPE;

/// I/O pin sense type
typedef enum __iopin_sense {
	IOPINSENSE_DISABLE,			//!< Disable pin sense
	IOPINSENSE_LOW_TRANSITION,	//!< Event on falling edge
	IOPINSENSE_HIGH_TRANSITION,	//!< Event on raising edge
	IOPINSENSE_TOGGLE,			//!< Event on state change
} IOPINSENSE;

/// I/O pin drive strength
typedef enum __iopin_drive_strength {
	IOPINSTRENGTH_REGULAR,		//!< Regular driver strength (normal default)
	IOPINSTRENGTH_STRONG,		//!< Stronger drive strength
} IOPINSTRENGTH;

#pragma pack(push,4)

/// I/O pin configuration data
typedef struct __iopin_cfg {
	int 		PortNo;			//!< Port number
	int 		PinNo;			//!< Pin number
	int 		PinOp;			//!< Pin function select index from 0, MCU dependent
	IOPINDIR	PinDir;			//!< Pin direction
	IOPINRES 	Res;			//!< Pin resistor setting
	IOPINTYPE	Type;			//!< I/O type
} IOPINCFG;

#pragma pack(pop)

/**
 * @brief	I/O pin event callback
 *
 * @param	IntNo : Interrupt number to which the I/O pin sense was assigned to.
 */
typedef void (*IOPINEVT_CB)(int IntNo);

#ifdef 	__cplusplus
extern "C" {
#endif

/**
 * @brief Configure individual I/O pin.
 *
 * This function is MCU dependent. Needs to be implemented per MCU
 *
 * @Param 	PortNo	: Port number
 * @Param	PinNo  	: Pin number
 * @Param	PinOp	: Pin function index from 0. MCU dependent
 * @Param	Dir     : I/O direction
 * @Param	Resistor : Resistor config
 * @Param	Type 	: I/O type
 */
void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type);

/**
 * @brief	Configure I/O pin with IOPIN_CFG data structure. Can be used for batch configuration
 *
 * @param	pCfg   : Pointer to an array gpio pin configuration
 * @Param	NbPins : Number of gpio pins to configure
 */
static inline void IOPinCfg(const IOPINCFG *pCfg, int NbPins) {
	if (pCfg == NULL || NbPins <= 0)
		return;

	for (int i = 0; i < NbPins; i++)
	{
		IOPinConfig(pCfg[i].PortNo, pCfg[i].PinNo, pCfg[i].PinOp, pCfg[i].PinDir,
					pCfg[i].Res, pCfg[i].Type);
	}
}

/**
 * @brief	Disable I/O pin
 *
 * Some hardware such as low power mcu allow I/O pin to be disconnected
 * in order to save power. There is no enable function. Reconfigure the
 * I/O pin to re-enable it.
 *
 * @param	PortNo 	: Port number
 * @param	PinNo	: Pin Number
 */
void IOPinDisable(int PortNo, int PinNo);

/**
 * @brief	Disable I/O pin sense interrupt
 *
 * @param	IntNo : Interrupt number to disable
 */
void IOPinDisbleInterrupt(int IntNo);

/**
 * @brief Enable I/O pin sensing interrupt event
 *
 * Generate an interrupt when I/O sense a state change.
 * The IntNo (interrupt number) parameter is processor dependent. Some is
 * directly the hardware interrupt number other is just an index in an array
 *
 *
 * @param	IntNo	: Interrupt number.
 * @Param	IntPrio : Interrupt priority
 * @Param	PortNo  : Port number (up to 32 ports)
 * @Param	PinNo   : Pin number (up to 32 pins)
 * @Param	Sense   : Sense type of event on the I/O pin
 * @Param	pEvtCB	: Pointer to callback function when event occurs
 *
 * @return	true - success
 */
bool IOPinEnableInterrupt(int IntNo, int IntPrio, int PortNo, int PinNo, IOPINSENSE Sense, IOPINEVT_CB pEvtCB);

/**
 * @brief Set I/O pin sensing option
 *
 * Some hardware allow pin sensing to wake up or active other subsystem without
 * requiring enabling interrupts. This requires the I/O already configured
 *
 * @param	PortNo : Port number (up to 32 ports)
 * @Param	PinNo   : Pin number (up to 32 pins)
 * @Param	Sense   : Sense type of event on the I/O pin
 */
void IOPinSetSense(int PortNo, int PinNo, IOPINSENSE Sense);

/**
 * @brief Set I/O pin drive strength option
 *
 * Some hardware allow setting pin drive strength. This requires the I/O already configured
 *
 * @param	PortNo 	: Port number (up to 32 ports)
 * @Param	PinNo  	: Pin number (up to 32 pins)
 * @Param	Strength: Pin drive strength
 */
void IOPinSetStrength(int PortNo, int PinNo, IOPINSTRENGTH Strength);

#ifdef __cplusplus
}
#endif

#endif	// __IOPINCFG_H__
