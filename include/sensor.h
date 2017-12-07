/*--------------------------------------------------------------------------
File   : sensor.h

Author : Hoang Nguyen Hoan          			Oct. 18, 2017

Desc   : Generic sensor abstraction


Copyright (c) 2017, I-SYST inc., all rights reserved

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
#ifndef __SENSOR_H__
#define __SENSOR_H__

#include <stdint.h>
#include <string.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "iopincfg.h"
#include "device.h"
#include "timer.h"

//
// Sensor operating mode
//
// Not all sensor devices have CONTINUOUS mode. If not avail, it can be implemented
// using timer with SINGLE mode.
//
// NOTE: Timer configuration and operation is handled by user firmware application
//
typedef enum __Sensor_OpMode {
	SENSOR_OPMODE_SINGLE,			// Single capture
	SENSOR_OPMODE_CONTINUOUS		// Continuous capture
} SENSOR_OPMODE;

//
// Sensor state
//
// To indicate current state of the sensor.
//
typedef enum __Sensor_State {
	SENSOR_STATE_SLEEP,				// Sleep state low power
	SENSOR_STATE_IDLE,				// Idle state powered on
	SENSOR_STATE_SAMPLING			// Sampling in progress
} SENSOR_STATE;

#ifdef __cplusplus

//
// Sensor generic base class
//
// Require implementations :
//	bool StartSampling();
//
// Require implementations from Device base class
//	bool Enable();
//	void Disable();
//	void Reset();
//
class Sensor : virtual public Device {
public:
	//
	// *** Require implementations ***
	//

	/**
	 * @brief	Start sampling data
	 *
	 * @return	true - success
	 */
	virtual bool StartSampling() = 0;

	//
	// *** Optional overloadable ***
	//

	/**
	 * @brief Set operating mode
	 *		sensor implementation must overload this function to do necessary
	 *	hardware setting for the operating mode.
	 *
	 * @param OpMode : Operating mode
	 * 					- SENSOR_OPMODE_SINGLE
	 * 					- SENSOR_OPMODE_CONTINUOUS
	 * @param Freq : Sampling frequency in Hz for continuous mode
	 *
	 * @return true- if success
	 */
	virtual bool Mode(SENSOR_OPMODE OpMode, uint32_t Freq) {
		vOpMode = OpMode;
		vSampFreq = Freq;

		return true;
	}

	/**
	 * @brief	Get current operating mode
	 *
	 * @return	Operating mode
	 * 				- SENSOR_OPMODE_SINGLE
	 * 				- SENSOR_OPMODE_CONTINUOUS
	 */
	virtual SENSOR_OPMODE Mode() { return vOpMode; }

	/**
	 * @brief	Get sampling frequency.
	 * 		The sampling frequency is relevant only in continuous mode
	 *
	 * @return	Frequency in Hz
	 */
	virtual uint32_t SamplingFrequency() { return vSampFreq; }

	/**
	 * @brief	Set sampling frequency.
	 * 		The sampling frequency is relevant only in continuous mode.
	 *
	 * @return	Frequency in Hz
	 */
	virtual uint32_t SamplingFrequency(uint32_t FreqHz) = 0;

	/**
	 * @brief	Set current sensor state
	 *
	 * @param 	State
	 *				- SENSOR_STATE_SLEEP	// Sleep state low power
	 *				- SENSOR_STATE_IDLE		// Idle state powered on
	 *				- SENSOR_STATE_SAMPLING	// Sampling in progress
	 *
	 * @return	Actual state. In the case where the new state could
	 * 			not be set, it returns the actual state of the sensor.
	 */
	virtual SENSOR_STATE State(SENSOR_STATE State) {
		vState = State;
		return vState;
	}

	/**
	 * @brief	Get current sensor state
	 *
	 * @return	Current state
	 *				- SENSOR_STATE_SLEEP	// Sleep state low power
	 *				- SENSOR_STATE_IDLE		// Idle state powered on
	 *				- SENSOR_STATE_SAMPLING	// Sampling in progress
	 */
	virtual SENSOR_STATE State() { return vState; }

protected:

	SENSOR_STATE vState;		// Current sensor state
	SENSOR_OPMODE vOpMode;		// Current operating mode
	uint32_t vSampFreq;			// Sampling frequency in Hz, relevant to CONTINUOUS mode
	Timer *vpTimer;				// Timer to use for time stamping data
	uint64_t vSampleCnt;		// Keeping sample count
};

extern "C" {
#endif	// __cplusplus

#ifdef __cplusplus
}

#endif	// __cplusplus

#endif	// __SENSOR_H__
