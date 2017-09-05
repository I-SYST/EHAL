/*--------------------------------------------------------------------------
File   : adc_nrf52.h

Author : Hoang Nguyen Hoan          June 16, 2017

Desc   : ADC implementation for Nordic nRF52 

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
#ifndef __ADC_NRF52_H__
#define __ADC_NRF52_H__

#include <stdint.h>

#include "nrf.h"

#include "converters/adc_device.h"

#define SAADC_NRF52_MAX_CHAN        8   // Max number of channels

extern "C" void SAADC_IRQHandler();

class ADCnRF52 : public ADCDevice {
public:
	ADCnRF52();
	virtual ~ADCnRF52();
	ADCnRF52(ADCnRF52&);	// ctor not allowed

	/**
	 * @brief	ADC device initialization
	 *
	 * @param	Cfg 	: Configuration data
	 * 			pIntrf	: Pointer to device interface instance
	 * 					  NULL, if self interface or internal SoC
	 * 					  such as MCU ADC pins
	 *
	 * @return	True - Success
	 */
	virtual bool Init(const ADC_CFG &Cfg, DeviceIntrf *pIntrf);

	/**
	 * @brief	Set conversion rate for continuous mode only
	 *
	 * @param 	Val : Rate value in Hz
	 *
	 * @return	Real rate value set in Hz
	 */
	virtual uint32_t Rate(uint32_t Val);

	/**
	 * @brief	Set conversion resolution
	 *
	 * @param 	Val : Resolution value in bits
	 *
	 * @return	Real resolution value set in bits
	 */
	virtual uint16_t Resolution(uint16_t Val);

	/**
	 * @brief	Configure channel for ADC conversion
	 *
	 * @param	pChanCfg : Array of channels to configure
	 * 			NbChan	 : Number of channels (array size)
	 *
	 * @return	True - Success
	 */
	virtual bool OpenChannel(const ADC_CHAN_CFG *pChanCfg, int NbChan);

	/**
	 * @brief	Close ADC channel
	 *
	 * @param 	Chan : Channel number
	 */
	virtual void CloseChannel(int Chan);

	/**
	 * @brief	Start ADC conversion process
	 *
	 * @return	True - Success
	 */
	virtual bool StartConversion();

	/**
	 * @brief	Stop ADC conversion
	 */
	virtual void StopConversion();

	/**
	 * @brief	Read converted data
	 *
	 * @param	pBuff : Buffer to receive converted data
	 * 			Len	  : Size of buffer array (total number of elements)
	 *
	 * @return	Number of ADC data in array.
	 */
	virtual int Read(ADC_DATA *pBuff, int Len);

	/**
	 * @brief	Execute auto calibration
	 *
	 * @return	true - success
	 */
	virtual bool Calibrate();

	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();

private:
	HCFIFO	vhFifo;

    friend void SAADC_IRQHandler();
};

#endif // __ADC_NRF52_H__
