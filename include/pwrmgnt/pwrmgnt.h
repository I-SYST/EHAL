/**-------------------------------------------------------------------------
@file	pwrmgnt.h

@brief	Generic power management definition

This file contains generic definitions to implement power management drivers
such as a PMIC chip or MCU builtin power management

@author	Hoang Nguyen Hoan
@date	July 25, 2019

@license

Copyright (c) 2019, I-SYST, all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST, I-SYST inc. or its contributors may be used to endorse or
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
#ifndef __PWRMGNT_H__
#define __PWRMGNT_H__

#include <stdint.h>

#include "device_intrf.h"
#include "device.h"
#include "miscdev/led.h"

/** @addtogroup Power
  * @{
  */


#define PWRMGNT_VOUT_MAXCNT			4	//!< Max number of Vout

class PowerMgnt;

#pragma pack(push, 1)
typedef enum __Charge_Type {
	PWR_CHARGE_TYPE_NORMAL,
	PWR_CHARGE_TYPE_TRICKLE,
	PWR_CHARGE_TYPE_AUTO				//!< Auto select optimum charge by driver implementation
} PWR_CHARGE_TYPE;

typedef struct __Vout_Cfg {
	int32_t mVout;						//!< Output voltage in mV
	uint32_t mAlimit;					//!< Output current limit in mA
} PWR_VOUT_CFG;

typedef enum __PowerMgnt_Evt {
	PWREVT_CHARGE_FULL,					//!< Battery full
	PWREVT_LOW_BAT,						//!< Battery low
	PWREVT_OVER_HEAT,					//!< Over heat detected
	PWREVT_CHARGE_DETECTED				//!< Charge source detected
} PWREVT;

typedef struct __Bat_Profile {
	uint32_t OpVolt;		//!< Battery nominal operating voltage in mV
	uint32_t ChrgVolt;		//!< Battery charge voltage in mV
	uint32_t Capacity;		//!< Battery capacity in mAh
	uint32_t ThermBConst;	//!< Thermistor B constant
} BAT_PROFILE;

typedef void (*PWRMGNT_EVTCB)(PowerMgnt *pSelf, PWREVT Evt);

typedef struct __Power_Config {
	uint32_t DevAddr;					//!< Device address
	PWR_VOUT_CFG * const pVout;			//!< Pointer to V out settings
	size_t NbVout;						//!< Number of V out
	int32_t VEndChrg;					//!< End of charge voltage level in mV
	uint32_t ChrgCurr;					//!< Charge current in mA
	uint32_t ChrgTimeout;				//!< Charge timeout in minutes
	bool bIntEn;						//!< Interrupt enable
	int IntPrio;						//!< Interrupt priority
	LED_DEV * const pLed;
	int NbLed;
	PWRMGNT_EVTCB pEvtHandler;
} PWRCFG;

#pragma pack(pop)

class PowerMgnt : public Device {
public:
	virtual bool Init(const PWRCFG &Cfg, DeviceIntrf *pIntrf) = 0;

	/**
	 * @brief	Set output voltage
	 *
	 * If output voltage is zero, turn off the output.
	 *
	 * @param	VoutIdx : Zero based index of output source
	 * @param	mVolt : Output voltage in mV
	 * @param	mALimit : Output current limit in mA if available
	 * 						set to zero for max capacity
	 *
	 * @return	Actual output voltage in mV
	 *
	 */
	virtual int32_t SetVout(size_t VoutIdx, int32_t mVolt, uint32_t mALimit) = 0;

	/**
	 * @brief	Set battery charging
	 *
	 * If charge current is set to zero, charging is turned off
	 *
	 * @param	Type : Charging type
	 * @param	mVoltEoC : End of charge voltage in mV
	 * @param	mACurr : Charge current limit
	 * 					0 : Disable charge
	 *
	 * @return	Actual charge current set.
	 */
	virtual uint32_t SetCharge(PWR_CHARGE_TYPE Type, int32_t mVoltEoC, uint32_t mACurr) = 0;

	virtual bool Charging() { return false; }

	/**
	 * @brief	Interrupt handler
	 *
	 * Optional implementation to handle interrupt. This is device specific.
	 *
	 */
	virtual void IrqHandler() {}

protected:
	uint32_t vChrgCurr;		//!< Charge current
	PWRMGNT_EVTCB vpEvtHandler;

private:
//	LED_DEV vLed;			//!< Led active level
};

// Low battery event handler
typedef void (*FGLOWCB)(PowerMgnt * const pPwrMnt);

typedef struct __FuelGauge_Cfg {
	BAT_PROFILE &BatProf;		//!< Reference to battery profile
	FGLOWCB	BatLowHandler;		//!< Battery low event handler
} FUELGAUGE_CFG;

class FuelGauge : public Device {
public:
	virtual bool Init(const FUELGAUGE_CFG &Cfg, DeviceIntrf * const pIntrf, PowerMgnt * const pPwrMnt) = 0;

	/**
	 * @brief	Get battery level
	 *
	 * @return	Battery level in (0-100) %
	 */
	virtual uint8_t BatLevel() = 0;
	virtual int32_t BatTemperature() = 0;
	virtual int32_t BatVoltage() = 0;

protected:
	void LowBatAlert() { if (vBatLowHandler) vBatLowHandler(vpPwrMgnt); }

private:
	FGLOWCB	vBatLowHandler;		//!< Low battery event handler
	BAT_PROFILE vBetProfile;
	PowerMgnt * const vpPwrMgnt;
};

#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif

/** @} End of group Power */

#endif //__PWRMGNT_H__
