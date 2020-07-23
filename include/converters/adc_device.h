/**-------------------------------------------------------------------------
@file	adc_device.h

@brief	Generic ADC device


@author	Hoang Nguyen Hoan
@date	June 16, 2017

@license

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

----------------------------------------------------------------------------*/
#ifndef __ADC_DEVICE_H__
#define __ADC_DEVICE_H__

#include <stdint.h>

#include "cfifo.h"
#include "device.h"
#include "device_intrf.h"
#include "coredev/timer.h"

/** @addtogroup Converters
  * @{
  */

typedef enum __ADC_Reference_Voltage_Type {
	ADC_REFVOLT_TYPE_INTERNAL,		//!< Internal fixed voltage specific to each IC
	ADC_REFVOLT_TYPE_SUPPLY,		//!< Device supply voltage VDD
	ADC_REFVOLT_TYPE_EXTERNAL		//!< Using external reference source
} ADC_REFVOLT_TYPE;

typedef enum __ADC_Rejection_Mode {
	ADC_REJECT_MODE_NONE,		//!< No rejection
	ADC_REJECT_MODE_50HZ,		//!< 50 Hz line noise rejection
	ADC_REJECT_MODE_60HZ,		//!< 60 Hz line noise rejection
	ADC_REJECT_MODE_BOTH		//!< both 50 & 60 Hz line noise rejection
} ADC_REJECT_MODE;


typedef enum __ADC_Conversion_Mode {
    ADC_CONV_MODE_SINGLE,        //!< Single channel conversion individually
    ADC_CONV_MODE_CONTINUOUS     //!< Multiple channels conversion successively
} ADC_CONV_MODE;

typedef enum __ADC_Channel_Type {
	ADC_CHAN_TYPE_SINGLE_ENDED,
	ADC_CHAN_TYPE_DIFFERENTIAL
} ADC_CHAN_TYPE;

// ADC pin internal connection
// Not all ADC devices offer this
typedef enum __ADC_Pin_Conn {
	ADC_PIN_CONN_NONE,
	ADC_PIN_CONN_PULLUP,
	ADC_PIN_CONN_PULLDOWN,
	ADC_PIN_CONN_VDD		//!< VDD connection is not full. It depends on device, could be VDD/2 or other.
							//!< check device specs for real value
} ADC_PIN_CONN;

typedef enum __ADC_Events {
	ADC_EVT_UNKNOWN,
	ADC_EVT_DATA_READY
} ADC_EVT;

#pragma pack(push, 4)

//
// Defines reference voltage sources
//
typedef struct __ADC_Reference_Voltage {
	ADC_REFVOLT_TYPE	Type;		//!< Reference voltage type
	float				Voltage;	//!< Reference voltage value
	int					Pin;		//!< External reference input number
} ADC_REFVOLT;

typedef struct __ADC_Pin_Config {
	int PinNo;				//!<
	ADC_PIN_CONN Conn;		//!< Pin internal connection
} ADC_PIN_CFG;

typedef struct __ADC_Channel_Config {
	int 			Chan;				//!< Channel number
	int 			RefVoltIdx;			//!< Index to which ADC reference voltage in the array to use.
	ADC_CHAN_TYPE	Type;				//!< ADC channel type
	uint32_t 		Gain;				//!< Bit 0-7 : Fractional gain value (2 = 1/2, 3 = 1/3, ...)
	int 			AcqTime;			//!< Acquisition time usec
	bool 			BurstMode;			//!< Oversampling
	ADC_PIN_CFG 	PinP;				//!< Pin positive
	ADC_PIN_CFG 	PinN;				//!< Pin negative
	int				FifoMemSize;		//!< Total memory size for CFIFO, CFIFO is used with interrupt enabled mode
	uint8_t			*pFifoMem;			//!< pointer to memory for CFIFO
} ADC_CHAN_CFG;

class AdcDevice;	// Forward declare

//typedef void (*ADC_EVTCB)(AdcDevice *pDev, ADC_EVT Evt);

typedef struct __ADC_Config {
    ADC_CONV_MODE Mode;			//!< Conversion mode
	const ADC_REFVOLT *pRefVolt;//!< Pointer to reference voltage array.
								//!< Many ADC can have multiple reference voltage input
	int			NbRefVolt;		//!< Total number of reference voltage input
	uint8_t		DevAddr;		//!< Device address. For example I2C device address
	int			Resolution;		//!< Sampling resolution in bits
	int			Rate;			//!< Sampling rate in mHz
	int			OvrSample;		//!< Oversample
	bool		bInterrupt;		//!< Enable/Disable interrupt
	int			IntPrio;		//!< Interrupt priority
	DEVEVTCB	EvtHandler;		//!< Device event handler
} ADC_CFG;

typedef struct __ADC_Data_Packet {
	uint32_t Timestamp;		//!< Time stamp base on conversion rate in continuous mode, 0 if SINGLE
	int Chan;				//!< Channel number
	float Data;				//!< Converted data in Volt
} ADC_DATA;

#pragma pack(pop)

/// ADC generic base class. implementation must derive from this class.
class AdcDevice : public Device {
public:

	/**
	 * @brief	ADC device initialization
	 *
	 * @param	Cfg 	: Configuration data
	 * @param	pTimer	: Pointer to timer object for time stamping if available
	 * @param	pIntrf	: Pointer to device interface instance
	 * 					  NULL, if self interface or internal SoC
	 * 					  such as MCU ADC pins
	 *
	 * @return
	 * 			- True	: Success
	 * 			- false	: Failed
	 */
	virtual bool Init(const ADC_CFG &Cfg, Timer * const pTimer, DeviceIntrf * const pIntrf) = 0;

	/**
	 * @brief	Set conversion rate for continuous mode only
	 *
	 * @param 	Val : Rate value in Hz
	 *
	 * @return	Real rate value set in Hz
	 */
	virtual uint32_t Rate(uint32_t Val) { vRate = Val; return vRate; }

	/**
	 * @brief	Get conversion rate for continuous mode only
	 *
	 * @return	Rate in Hz
	 */
	virtual uint32_t Rate() { return vRate; }

	/**
	 * @brief	Set conversion resolution
	 *
	 * @param 	Val : Resolution value in bits
	 *
	 * @return	Real resolution value set in bits
	 */
	virtual uint16_t Resolution(uint16_t Val) { vResolution = Val; return vResolution; }

	/**
	 * @brief 	Get current conversion resolution
	 *
	 * @return	Resolution in bits
	 */
	virtual uint16_t Resolution() { return vResolution; }

	/**
	 * @brief	Configure channel for ADC conversion
	 *
	 * @param	pChanCfg : Array of channels to configure
	 * @param	NbChan	 : Number of channels (array size)
	 *
	 * @return	True - Success
	 */
	virtual bool OpenChannel(const ADC_CHAN_CFG * const pChanCfg, int NbChan) = 0;

	/**
	 * @brief	Close ADC channel
	 *
	 * @param 	Chan : Channel number
	 */
	virtual void CloseChannel(int Chan) = 0;

	/**
	 * @brief	Start ADC conversion process
	 *
	 * @return	True - Success
	 */
	virtual bool StartConversion() = 0;

	/**
	 * @brief	Stop ADC conversion
	 */
	virtual void StopConversion() = 0;

	/**
	 * @brief	Read data from device if available
	 *
	 * @return	true if new data is available
	 */
	virtual bool UpdateData() = 0;

	/**
	 * @brief	Read converted data multiple channels
	 *
	 * @param	pBuff : Buffer to receive converted data
	 * @param	Len	  : Size of buffer array (total number of elements)
	 *
	 * @return	Number of ADC data in array.
	 */
	virtual int Read(ADC_DATA *pBuff, int Len) = 0;

	/**
	 * @brief	Read ADC data of one channel
	 *
	 * @param 	Chan : Channel number
	 * @param	pBuff : Pointer to buffer for returning data
	 *
	 * @return	true - data available
	 */
	virtual bool Read(int Chan, ADC_DATA *pBuff) = 0;

	/**
	 * @brief	Execute auto calibration
	 *
	 * @return	true - success
	 */
	virtual bool Calibrate() = 0;

	virtual ADC_CONV_MODE Mode() { return vMode; }
	virtual void Mode(ADC_CONV_MODE Mode) { vMode = Mode; }

protected:
	//void SetEvtHandler(ADC_EVTCB EvtHandler) { vEvtHandler = EvtHandler; }
	void SetRefVoltage(const ADC_REFVOLT * const pRefVolt, int NbRefVolt) {
		vpRefVolt = pRefVolt;
		vNbRefVolt = NbRefVolt;
	}
/*
	void EvtHandler(ADC_EVT Evt) {
		if (vEvtHandler)
			vEvtHandler(this, Evt);
	}
*/
	const ADC_REFVOLT *vpRefVolt;	//!< pointer to array of predefined reference voltages
	int vNbRefVolt;					//!< number of reference voltages (array size)
	uint16_t vResolution;			//!< Resolution in bits
	uint32_t vRate;					//!< Sampling rate in Hz
	ADC_CONV_MODE vMode;			//!< Conversion mode single/continuous
	bool vbInterrupt;				//!< Interrupt enable flag
	uint32_t vIntPrio;				//!< Interrupt priority

private:
//	ADC_EVTCB vEvtHandler;
};

/** @} End of group Converters */

#endif // __ADC_DEVICE_H__
