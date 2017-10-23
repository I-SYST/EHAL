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
typedef enum __Sensor_OpMode {
	SENSOR_OPMODE_SLEEP,
	SENSOR_OPMODE_SINGLE,			// Single capture
	SENSOR_OPMODE_CONTINUOUS		// Continuous capture
} SENSOR_OPMODE;

#ifdef __cplusplus

class Sensor : virtual public Device {
public:
	/**
	 * @brief Set operating mode
	 *		sensor implementation must overload this function to do necessary
	 *	hardware setting for the operating mode.
	 *
	 * @param OpMode : Operating mode
	 * 					- SENSOR_OPMODE_SLEEP
	 * 					- SENSOR_OPMODE_SINGLE
	 * 					- SENSOR_OPMODE_CONTINUOUS
	 * @param Freq : Sampling frequency in Hz for continuous mode
	 *
	 * @return true- if success
	 */
	virtual bool SetMode(SENSOR_OPMODE OpMode, uint32_t Freq) {
		vOpMode = OpMode;
		vSampFreq = Freq;

		return true;
	}

	/**
	 * @brief	Start sampling data
	 *
	 * @return	true - success
	 */
	virtual bool StartSampling() = 0;

	/**
	 * @brief	Get current operating mode
	 *
	 * @return	Operating mode
	 * 				- SENSOR_OPMODE_SLEEP
	 * 				- SENSOR_OPMODE_SINGLE
	 * 				- SENSOR_OPMODE_CONTINUOUS
	 */
	SENSOR_OPMODE Mode() { return vOpMode; }
	uint32_t SamplingFrequency() { return vSampFreq; }

protected:
	SENSOR_OPMODE vOpMode;
	uint32_t vSampFreq;			// Sampling frequency in Hz
	Timer *vpTimer;				// Timer to use for time stamping data
};

extern "C" {
#endif	// __cplusplus

#ifdef __cplusplus
}

#endif	// __cplusplus

#endif	// __SENSOR_H__
