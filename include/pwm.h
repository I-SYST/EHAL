/**-------------------------------------------------------------------------
@file	pwm.h

@brief	Generic implementation of PWM (Pulse Width Modulation)

@author	Hoang Nguyen Hoan
@date	May 15, 2018

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
#ifndef __PWM_H__
#define __PWM_H__

#include <stdint.h>

typedef enum __Pwm_Event {
	PWM_EVT_STOPPED,
} PWM_EVT;

typedef enum __Pwm_Mode {
	PWM_MODE_EDGE,				//!< Edge --____
	PWM_MODE_CENTER				//!< Center __--__ (only if hardware is supported)
} PWM_MODE;

typedef enum __Pwm_Polarity {
	PWM_POL_LOW,				//!< Active low __----, --__--
	PWM_POL_HIGH				//!< Active high --____, __--__
} PWM_POL;

typedef struct __Pwm_Device PWM_DEV;

typedef void (*PWMEVTHANDLER) (PWM_DEV *pDev, PWM_EVT Evt);

/// PWM configuration data
typedef struct __Pwm_Config {
	int DevNo;					//!< PWM device number 0 based index
	uint32_t Freq;				//!< PWM frequency in hertz
	PWM_MODE Mode;				//!< PWM mode edge or center
	bool bIntEn;				//!< Enable interrupt
	int IntPrio;				//!< Interrupt prio
	PWMEVTHANDLER pEvtHandler;	//!< Event handler
} PWM_CFG;

/// PWM channel configuration
typedef struct __Pwm_Chan_Cfg {
	int Chan;					//!< zero based channel number
	PWM_POL Pol;				//!< Polarity 0 - active 0, 1 - active high
	int Port;					//!< output port number
	int Pin;					//!< output pin number
} PWM_CHAN_CFG;

// Private driver data for internal use only
struct __Pwm_Device {
	int DevNo;
	uint32_t Freq;
	PWM_MODE Mode;				//!< PWM mode edge or center
	PWMEVTHANDLER pEvtHandler;
	void *pDevData;				//!< Internal implementation data

};


#ifdef __cplusplus
extern "C" {
#endif

bool PWMInit(PWM_DEV *pDev, const PWM_CFG *pCfg);
bool PWMEnable(PWM_DEV *pDev);
void PWMDisable(PWM_DEV *pDev);
bool PWMOpenChannel(PWM_DEV *pDev, const PWM_CHAN_CFG *pChanCfg, int NbChan);
void PWMCloseChannel(PWM_DEV *pDev, int Chan);
bool PWMStart(PWM_DEV *pDev, uint32_t msDur);
void PWMStop(PWM_DEV *pDev);
bool PWMSetFrequency(PWM_DEV *pDev, uint32_t Freq);
bool PWMSetDutyCycle(PWM_DEV *pDev, int Chan, int Percent);

#ifdef __cplusplus
}

/// PWM generic base class
class Pwm {
public:

	/**
	 * @brief	Initialize PWM device
	 *
	 * @param 	Cfg	: Configuration data
	 *
	 * @return	true - success
	 */
	virtual bool Init(const PWM_CFG &Cfg) { return PWMInit(&vDev, &Cfg); }

	/**
	 * @brief	Enable PWM device
	 *
	 * @return	true - success
	 */
	virtual bool Enable() { return PWMEnable(&vDev); }

	/**
	 * @brief	Dsiable PWM device
	 */
	virtual void Disable() { PWMDisable(&vDev); }

	/**
	 * @brief	Open PWM channel
	 * @param 	pChanCfg	: Channel configuration data
	 * @param 	NbChan		: Nb of channel to open
	 *
	 * @return	true - success
	 */
	virtual bool OpenChannel(const PWM_CHAN_CFG *pChanCfg, int NbChan) { return PWMOpenChannel(&vDev, pChanCfg, NbChan); }

	/**
	 * @brief	Close channel
	 *
	 * @param 	Chan	: Channel number to close
	 */
	virtual void CloseChannel(int Chan) { PWMCloseChannel(&vDev, Chan); }

	/**
	 * @brief	Start PWM
	 *
	 * @param 	msDur	: Duration in msec
	 *
	 * @return	true - success
	 */
	virtual bool Start(uint32_t msDur = 0) { return PWMStart(&vDev, msDur); }

	/**
	 * @brief	Stop PWM
	 */
	virtual void Stop() { PWMStop(&vDev); }

	/**
	 * @brief	Set PWM frequency
	 *
	 * @param 	Freq	: Frequency in hertz
	 *
	 * @return	true - success
	 */
	virtual bool SetFrequency(uint32_t Freq) { return PWMSetFrequency(&vDev, Freq); }

	/**
	 * @brief	Set duty cycle
	 *
	 * @param 	Chan	: Channel number to change duty cycle
	 * @param 	Percent	: Duty cycle in % (0-100%)
	 *
	 * @return	true - success
	 */
	virtual bool SetDutyCycle(int Chan, int Percent) { return PWMSetDutyCycle(&vDev, Chan, Percent); }

private:
	PWM_DEV vDev;
};

#endif

#endif // __PWM_H__
