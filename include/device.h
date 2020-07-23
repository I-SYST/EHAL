/**-------------------------------------------------------------------------
@file	device.h

@brief	Generic device base class

This is the base class to implement all sort devices, hardware or software.
For example a sensor device or a software audio/video decoder.
The device can transfer data via it's DeviceIntrf object.

Important NOTE : For performance, there is no pointer or
parameter validation at this low level layer.  It is the responsibility of
caller to pre-validate all access

@author	Hoang Nguyen Hoan
@date	Feb. 12, 2017

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
#ifndef __DEVICE_H__
#define __DEVICE_H__

#include <stdint.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "coredev/iopincfg.h"
#include "coredev/timer.h"
#include "device_intrf.h"

/// @brief	Defines interrupt pin polarity of the device.
///
/// Many hardware devices can have interrupt pin polarity configurable.
typedef enum __Dev_Interrupt_Polarity {
	DEVINTR_POL_LOW,	//!< Interrupt pin active low
	DEVINTR_POL_HIGH	//!< Interrupt pin active high
} DEVINTR_POL;

typedef enum __Device_Event {
	DEV_EVT_DATA_RDY
} DEV_EVT;

#ifdef __cplusplus

class Device;

typedef void (*DEVEVTCB)(Device * const pDev, DEV_EVT Evt);

/// @brief	Device base class.
///
/// This is the base class to implement all sort devices, hardware or software.
/// For example a sensor device or a software audio decoder.  The device can transfer
/// data via it's DeviceIntrf object.
/// Important NOTE : For performance, there is no pointer or
/// parameter validation at this low level layer.  It is the responsibility of
/// caller to pre-validate all access
class Device {
public:
	Device();
	virtual ~Device() {}

	//
	// *** Require implementations ***
	//

	/**
	 * @brief	Power on or wake up device
	 *
	 * @return	true - If success
	 */
	virtual bool Enable() = 0;

	/**
	 * @brief	Put device in power down or power saving sleep mode
	 *
	 * This function is used to put the device in lowest power mode
	 * possible so that the Enable function can wake up without full
	 * initialization.
	 */
	virtual void Disable() = 0;

	/**
	 * @brief	Reset device to it initial default state
	 */
	virtual void Reset() = 0;

	//
	// *** Optional implementations ***
	//

	/**
	 * @brief	Power off the device completely.
	 *
	 * If supported, this will put the device in complete power down.
	 * Full re-initialization is required to re-enable the device.
	 */
	virtual void PowerOff() {}

	/**
	 * @brief	Set device's map address
	 *
	 * Device address is dependent of interface and device type. For I2C type it
	 * would be the 7 bits address, SPI would be CS pin index, other memory mapped
	 * would be a 32bit address.
	 *
	 * @param 	Addr : Device's address or zero based chip select index
	 */
	virtual void DeviceAddress(uint32_t Addr) { vDevAddr =  Addr; }

	/**
	 * @brief	Get device's map address
	 *
	 * @return	Address or chip select pin zero based index
	 */
	virtual uint32_t DeviceAddress() { return vDevAddr; }

	/**
	 * @brief	Get device id.
	 *
	 * This device id value is implementation specific.  It can store hardware
	 * device identifier or serial number at the discretion of the implementor
	 *
	 * @return	64 Bits device id value
	 */
	virtual uint64_t DeviceID() { return vDevId; }

	/**
	 * @brief	Read device's register/memory block.
	 *
	 * This default implementation sets bit 7 of the Cmd/Addr byte for SPI read access as most
	 * devices work this way on SPI interface. Overwrite this implementation if SPI access is different
	 *
	 * @param 	pCmdAddr 	: Buffer containing command or address to be written
	 * 						  prior reading data back
	 * @param	CmdAddrLen 	: Command buffer size
	 * @param	pBuff		: Data buffer container
	 * @param	BuffLen		: Data buffer size
	 *
	 * @return	Actual number of bytes read
	 */
	virtual int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen);

	/**
	 * @brief	Write to device's register/memory block
	 *
	 * This default implementation clears bit 7 of the Cmd/Addr byte for SPI write access as most
	 * devices work this way on SPI interface.  Overwrite this implementation if SPI access is different
	 *
	 * @param 	pCmdAddr 	: Buffer containing command or address to be written
	 * 						  prior writing data back
	 * @param	CmdAddrLen 	: Command buffer size
	 * @param	pData		: Data buffer to be written to the device
	 * @param	DataLen		: Size of data
	 *
	 * @return	Actual number of bytes written
	 */
	virtual int Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen);

	/**
	 * @brief	Read device's 8 bits register/memory
	 *
	 * @param 	pRegAddr	: Buffer containing address location to read
	 * @param	RegAddrLen	: Address buffer size
	 *
	 * @return	Data read
	 */
	virtual uint8_t Read8(uint8_t *pRegAddr, int RegAddrLen) {
		uint8_t val = 0;
		Read(pRegAddr, RegAddrLen, &val, 1);
		return val;
	}

	/**
	 * @brief	Read device's 16 bits register/memory
	 *
	 * @param 	pRegAddr	: Buffer containing address location to read
	 * @param	RegAddrLen	: Address buffer size
	 *
	 * @return	Data read
	 */
	virtual uint16_t Read16(uint8_t *pRegAddr, int RegAddrLen) {
		uint16_t val = 0;
		Read(pRegAddr, RegAddrLen,(uint8_t*) &val, 2);
		return val;
	}

	/**
	 * @brief	Read device's 32 bit register/memory
	 *
	 * @param 	pRegAddr	: Buffer containing address location to read
	 * @param	RegAddrLen	: Address buffer size
	 *
	 * @return	Data read
	 */
	virtual uint32_t Read32(uint8_t *pRegAddr, int RegAddrLen) {
		uint32_t val = 0;
		Read(pRegAddr, RegAddrLen, (uint8_t*)&val, 4);
		return val;
	}

	/**
	 * @brief	Write 8 bits data to device's register/memory
	 *
	 * @param 	pRegAddr	: Buffer containing address location to write
	 * @param	RegAddrLen	: Address buffer size
	 * @param	Data		: Data to be written to the device
	 *
	 * @return	true - Success
	 */
	virtual bool Write8(uint8_t *pRegAddr, int RegAddrLen, uint8_t Data) {
		return Write(pRegAddr, RegAddrLen, &Data, 1) > 0;
	}

	/**
	 * @brief	Write 16 bits data to device's register/memory
	 *
	 * @param 	pRegAddr	: Buffer containing address location to write
	 * @param	RegAddrLen	: Address buffer size
	 * @param	Data		: Data to be written to the device
	 *
	 * @return	true - Success
	 */
	virtual bool Write16(uint8_t *pRegAddr, int RegAddrLen, uint16_t Data) {
		return Write(pRegAddr, RegAddrLen, (uint8_t*)&Data, 2) > 1;
	}

	/**
	 * @brief	Write 32 bits data to device's register/memory
	 *
	 * @param 	pRegAddr	: Buffer containing address location to write
	 * @param	RegAddrLen	: Address buffer size
	 * @param	Data		: Data to be written to the device
	 *
	 * @return	true - Success
	 */
	virtual bool Write32(uint8_t *pRegAddr, int RegAddrLen, uint32_t Data) {
		return Write(pRegAddr, RegAddrLen, (uint8_t*)&Data, 1) > 3;
	}

	/**
	 * @brief	Return availability of the device
	 *
	 * This function return true if the device has been detected and ready to use.
	 *
	 * @return	true - Device is valid.
	 */
	bool Valid() { return vbValid; }

	DEVINTRF_TYPE InterfaceType() { return vpIntrf != nullptr ? vpIntrf->Type() : DEVINTRF_TYPE_UNKOWN; }

	/**
	 * @brief	Get timer pointer used for timestamping
	 *
	 * @return	Pointer to Timer object.
	 * 			Never delete the returned pointer.  This is for embedded system.
	 * 			Normally objects are static not dynamically allocated
	 */
	virtual operator Timer * const () { return vpTimer; }	// Get device interface data (handle)

	void SetEvtHandler(DEVEVTCB EvtHandler) { vEvtHandler = EvtHandler; }
	virtual void EvtHandler(DEV_EVT Evt) { if (vEvtHandler) vEvtHandler(this, Evt); }

protected:

	/**
	 * @brief	Store device id.
	 *
	 * This device id value is implementation specific.  It can store hardware
	 * device identifier or serial number at the discretion of the implementor
	 *
	 * @param	DevId : Device id value to store
	 */
	void DeviceID(uint64_t DevId) { vDevId = DevId; }

	/**
	 * @brief	Set device validity.
	 *
	 * Set valid to true if device is detect and initialized.  Otherwise set it to false
	 *
	 */
	void Valid(bool bVal) { vbValid = bVal; }

	/**
	 * @brief	Set device's communication interface
	 *
	 * @param 	pIntrf : Pointer to preinitialized static interface.
	 */
	void Interface(DeviceIntrf *pIntrf) { vpIntrf = pIntrf; }

	/**
	 * @brief	Get device's communication interface
	 *
	 * @return	return pointer to interface in use or NULL
	 */
	DeviceIntrf *Interface() { return vpIntrf; }

	void InterruptEnabled(bool En) { vbIntEn = En; }
	bool InterruptEnabled() { return vbIntEn; }

	bool		vbValid;		//!< Device is valid ready to use (passed detection)
	uint32_t 	vDevAddr;		//!< Device address or chip select index
	DeviceIntrf *vpIntrf;		//!< Device's interface
	Timer 		*vpTimer;		//!< Timer to use for time stamping data or create a timer event
	uint64_t	vDevId;			//!< This is implementation specific data for device identifier
	 	 	 	 	 	 	 	//!< could be value read from hardware register or serial number
	bool 		vbIntEn;		//!< Interrupt enabled
	DEVEVTCB 	vEvtHandler;	//!< Event handler callback
};

extern "C" {
#endif	// __cplusplus


#ifdef __cplusplus
}

#endif	// __cplusplus

#endif	// __DEVICE_H__
