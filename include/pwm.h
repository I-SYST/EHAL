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
	PWM_EVT_STOPPED = 0,
	PWM_EVT_STARTED,
	PWM_EVT_PERIOD,
} PWM_EVT;

typedef enum __Pwm_Mode {
	PWM_MODE_EDGE,				//!< Edge --____
	PWM_MODE_CENTER				//!< Center __--__ (only if hardware is supported)
} PWM_MODE;

typedef enum __Pwm_Polarity {
	PWM_POL_LOW,				//!< Active low __----, --__--
	PWM_POL_HIGH				//!< Active high --____, __--__
} PWM_POL;

typedef struct __Pwm_Device PwmDev_t;
typedef PwmDev_t	PWM_DEV;

typedef void (*PwmEvtHandler_t) (PwmDev_t *pDev, PWM_EVT Evt);

/// PWM configuration data
typedef struct __Pwm_Config {
	int DevNo;					//!< PWM device number 0 based index
	uint32_t Freq;				//!< PWM frequency in hertz
	PWM_MODE Mode;				//!< PWM mode edge or center
	bool bIntEn;				//!< Enable interrupt
	int IntPrio;				//!< Interrupt prio
	PwmEvtHandler_t pEvtHandler;	//!< Event handler
} PwmCfg_t;

typedef PwmCfg_t	PWM_CFG;

/// PWM channel configuration
typedef struct __Pwm_Chan_Cfg {
	int Chan;					//!< zero based channel number
	PWM_POL Pol;				//!< Polarity 0 - active 0, 1 - active high
	int Port;					//!< output port number
	int Pin;					//!< output pin number
} PwmChanCfg_t;

typedef PwmChanCfg_t	PWM_CHAN_CFG;

// Private driver data for internal use only
struct __Pwm_Device {
	int DevNo;
	uint32_t Freq;
	PWM_MODE Mode;				//!< PWM mode edge or center
	PwmEvtHandler_t pEvtHandler;
	void *pDevData;				//!< Internal implementation data
};


#ifdef __cplusplus
extern "C" {
#endif

bool PWMInit(PwmDev_t *pDev, const PwmCfg_t *pCfg);
bool PWMEnable(PwmDev_t *pDev);
void PWMDisable(PwmDev_t *pDev);
bool PWMOpenChannel(PwmDev_t *pDev, const PwmChanCfg_t *pChanCfg, int NbChan);
void PWMCloseChannel(PwmDev_t *pDev, int Chan);
bool PWMStart(PwmDev_t *pDev, uint32_t msDur);
void PWMStop(PwmDev_t *pDev);
bool PWMSetFrequency(PwmDev_t *pDev, uint32_t Freq);
bool PWMSetDutyCycle(PwmDev_t *pDev, int Chan, int Percent);

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
	virtual bool Init(const PwmCfg_t &Cfg) { return PWMInit(&vDev, &Cfg); }

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
	virtual bool OpenChannel(const PwmChanCfg_t *pChanCfg, int NbChan) { return PWMOpenChannel(&vDev, pChanCfg, NbChan); }

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
	virtual bool Frequency(uint32_t Freq) { return PWMSetFrequency(&vDev, Freq); }

	/**
	 * @brief	Set duty cycle
	 *
	 * @param 	Chan	: Channel number to change duty cycle
	 * @param 	Percent	: Duty cycle in % (0-100%)
	 *
	 * @return	true - success
	 */
	virtual bool DutyCycle(int Chan, int Percent) { return PWMSetDutyCycle(&vDev, Chan, Percent); }

private:
	PwmDev_t vDev;
};

#endif

#endif // __PWM_H__
