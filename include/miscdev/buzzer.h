/**-------------------------------------------------------------------------
@file	buzzer.h

@brief	Generic implementation of buzzer driver

@author	Hoang Nguyen Hoan
@date	May 22, 2018

@license

Copyright (c) 2018, I-SYST, all rights reserved

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

#ifndef __BUZZER_H__
#define __BUZZER_H__

#include "pwm.h"

/** @addtogroup MiscDev
  * @{
  */

typedef struct __Buzzer_Device {
	PWM_DEV *pPwm;			//!< Pointer to external PWM interface
	int DutyCycle;			//!< PWM duty cycle value for volume
	int Chan;				//!< PWM Channel used for the buzzer
} BUZZER_DEV;

#ifdef __cplusplus

class Buzzer {
public:

	/**
	 * @brief	Buzzer initialization
	 *
	 * @param 	pPwm	: Pointer PWM interface connected to buzzer
	 * @param 	Chan	: PWM channel used for the buzzer
	 *
	 * @return	true - success
	 */
	virtual bool Init(Pwm * const pPwm, int Chan);

	/**
	 * @brief	Set buzzer volume
	 *
	 * @param 	Volume	: Volume in % (0-100)
	 */
	virtual void Volume(int Volume);

	/**
	 * @brief	Play frequency
	 *
	 * @param	Freq :
	 * @param	msDuration	: Play duration in msec.
	 *							if != 0, wait for it then stop
	 *							else let running and return (no stop)
	 */
	virtual void Play(uint32_t Freq, uint32_t msDuration);

	/**
	 * @brief	Stop buzzer
	 *
	 */
	virtual void Stop();

private:

	Pwm *vpPwm;			//!< Pointer to external PWM interface
	int vDutyCycle;		//!< PWM duty cycle value for volume
	int vChan;			//!< PWM Channel used for the buzzer
};

extern "C" {
#endif // __cplusplus

#ifdef __cplusplus
}
#endif	// __cplusplus

/** @} end group IMU */

#endif // __BUZZER_H__

