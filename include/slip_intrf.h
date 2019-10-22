/**-------------------------------------------------------------------------
@file	slip_intrf.h

@brief	Implementation of SLIP device interface

The Serial Line Internet Protocol (also SLIP) is an encapsulation of the
Internet Protocol designed to work over serial ports and router connections.
It is documented in RFC 1055.

@author	Hoang Nguyen Hoan
@date	Aug. 7, 2019

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#ifndef __SLIP_INTRF_H__
#define __SLIP_INTRF_H__

#include "device_intrf.h"

#define SLIP_END_CODE				0xC0

#define SLIP_BUFF_MAX				512

typedef struct __Slip_Device {
	DEVINTRF DevIntrf;					//!< This interface
	DEVINTRF *pPhyIntrf;				//!< Physical transport interface
	void *pObj;							//!< Slip object instance
} SLIPDEV;

#ifdef __cplusplus
extern "C" {
#endif

bool SlipInit(SLIPDEV * const pDev, DEVINTRF * const pPhyIntrf, bool bBlocking);
static inline int SlipGetRate(SLIPDEV * const pDev) { return DeviceIntrfGetRate(&pDev->DevIntrf); }
static inline int SlipSetRate(SLIPDEV * const pDev, int Rate) { return DeviceIntrfSetRate(&pDev->DevIntrf, Rate); }
static inline void SlipEnable(SLIPDEV * const pDev) { DeviceIntrfEnable(&pDev->DevIntrf); }
static inline void SlipDisable(SLIPDEV * const pDev) { DeviceIntrfDisable(&pDev->DevIntrf); }
static inline int SlipRx(SLIPDEV * const pDev, uint8_t *pBuff, int Bufflen) {
	return DeviceIntrfRx(&pDev->DevIntrf, 0, pBuff, Bufflen);
}
static inline int SlipTx(SLIPDEV * const pDev, uint8_t *pData, int Datalen) {
	return DeviceIntrfTx(&pDev->DevIntrf, 0, pData, Datalen);
}


#ifdef __cplusplus
}

class Slip : public DeviceIntrf {
public:
	bool Init(DeviceIntrf * const pIntrf, bool bBlocking = true);

	/**
	 * @brief	Operator to convert this class to device interface handle to be
	 * 			used with C functions.
	 *
	 * @return	Pointer to internal DEVINTRF to be used with C interface functions
	 */
	operator DEVINTRF * const () { return &vDevData.DevIntrf; }
	operator SLIPDEV * const () { return &vDevData; }

	/**
	 * @brief	Set data rate of the interface in Hertz.
	 *
	 * This is not a clock frequency but rather the transfer frequency (number
	 * of transfers per second). It has meaning base on the implementation as
	 * bits/sec or bytes/sec or whatever the case
	 *
	 * @param	DataRate : Data rate to be set in Hertz (transfer per second)
	 *
	 * @return 	Actual transfer rate per second set.  It is the real capable rate
	 * 			closest to rate being requested.
	 */
	int Rate(int DataRate) { return DeviceIntrfSetRate(*this, DataRate); }

	/**
	 * @brief	Get data rate of the interface in Hertz.
	 *
	 * This is not a clock frequency but rather the transfer frequency (number
	 * of transfers per second). It has meaning base on the implementation as
	 * bits/sec or bytes/sec or whatever the case
	 *
	 * @return	Transfer rate per second
	 */
	int Rate(void) { return DeviceIntrfGetRate(*this); }

    /**
     * @brief	Prepare start condition to receive data with subsequence RxData.
     *
     * This can be in case such as start condition for I2C or Chip Select for
     * SPI or precondition for DMA transfer or whatever requires it or not
     *
     * NOTE: This function must check & set the busy state for re-entrancy
     *
     * On success StopRx must be called to release busy flag
     *
     * @param	DevAddr   : The device selection id scheme
     *
     * @return 	true - Success\n
     * 			false - failed.
     */
	virtual bool StartRx(int DevAddr) { return DeviceIntrfStartRx(*this, DevAddr); }

	/**
	 * @brief	Receive data into pBuff passed in parameter.  Assuming StartRx was
	 * called prior calling this function to get the actual data
	 *
	 * @param	pBuff 	  : Pointer to memory area to receive data.
	 * @param	BuffLen   : Length of buffer memory in bytes
	 *
	 * @return	Number of bytes read
	 */
	virtual int RxData(uint8_t *pBuff, int BuffLen)  { return DeviceIntrfRxData(*this, pBuff, BuffLen); }

	// Stop receive
	// WARNING !!!!!
	// This functions MUST ONLY be called if StartRx returns true.
	/**
	 * @brief	Completion of read data phase.
	 *
	 * Do require post processing after data has been received via RxData
	 * This function must clear the busy state for re-entrancy.\n
	 * Call this function only if StartRx was successful.
	 */
	virtual void StopRx(void) { DeviceIntrfStopTx(*this); }

	// Initiate transmit
    // WARNING this function must be used in pair with StopTx
    // Re-entrance protection flag is used
    // On success, StopTx must be after transmission is completed to release flag
	/**
	 * @brief	Prepare start condition to transfer data with subsequence TxData.
	 *
	 * This can be in case such as start condition for I2C or Chip Select for
	 * SPI or precondition for DMA transfer or whatever requires it or not
	 *
	 * NOTE: This function must check & set the busy state for re-entrancy
	 *
	 * On success StopRx must be called to release busy flag
	 *
	 * @param	DevAddr   : The device selection id scheme
	 *
	 * @return 	true - Success\n
	 * 			false - failed
	 */
	virtual bool StartTx(int DevAddr) { return DeviceIntrfStartTx(*this, DevAddr); }

	/**
	 * @brief	Transfer data from pData passed in parameter.  Assuming StartTx was
	 * called prior calling this function to send the actual data
	 *
	 * @param	pData 	: Pointer to memory area of data to send.
	 * @param	DataLen : Length of data memory in bytes
	 *
	 * @return	Number of bytes sent
	 */
	virtual int TxData(uint8_t *pData, int DataLen) { return DeviceIntrfTxData(*this, pData, DataLen); }

	// Stop transmit
	// WARNING !!!!!
	// This functions MUST ONLY be called if StartTx returns true.
	/**
	 * @brief	Completion of sending data via TxData.
	 *
	 * Do require post processing
	 * after all data was transmitted via TxData.
	 *
	 * NOTE: This function must clear the busy state for re-entrancy
	 * Call this function only if StartTx was successful.
	 */
	virtual void StopTx(void) { DeviceIntrfStopTx(*this); }

private:
	SLIPDEV vDevData;
};

#endif

#endif // __SLIP_INTRF_H__
