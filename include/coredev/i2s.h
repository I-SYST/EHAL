/**-------------------------------------------------------------------------
@file	i2s.h

@brief	I2S (Inter-IC sound interface) generic definitions


@author	Nguyen Hoan Hoang
@date	Apr. 4, 2020

@license

MIT License

Copyright (c) 2020 I-SYST inc. All rights reserved.

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

#ifndef __I2S_H__
#define __I2S_H__

#include <stdint.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "cfifo.h"
#include "device_intrf.h"
#include "iopincfg.h"

#define I2S_MAX_NB_IOPIN		5

/// I2S operating modes
typedef enum __I2S_Mode {
	I2SMODE_MASTER,
	I2SMODE_SLAVE
} I2SMODE;

typedef enum __I2S_Sample_Width {
	I2SSWIDTH_8BITS = 8,
	I2SSWIDTH_16BITS = 16,
	I2SSWIDTH_24BITS = 24
} I2SSWIDTH;

typedef enum __I2S_Format {
	I2SFMT_NORMAL,		//!< I2S standard
	I2SFMT_ALIGN_LEFT,	//!< Left aligned
	I2SFMT_ALIGN_RIGHT	//!< Right aligned
} I2SFMT;

typedef enum __I2S_Channel {
	I2SCHAN_STEREO,
	I2SCHAN_LEFT,
	I2SCHAN_RIGHT
} I2SCHAN;

/// I2S pins map index
#define I2S_SCK_IOPIN_IDX		0	//!< Serial clock
#define I2S_WS_IOPIN_IDX		1	//!< Word select (left-right clock)
#define I2S_SDI_IOPIN_IDX		2	//!< Serial Data
#define I2S_SDO_IOPIN_IDX		3	//!< Serial data out
#define I2S_MCK_IOPIN_IDX		4	//!< Master clock

#pragma pack(push, 4)

/// Configuration data used to initialize device
typedef struct __I2S_Config {
	int DevNo;			//!< Device physical instance number
	const IOPINCFG *pIOPinMap;	//!< Define I/O pins used by I2S (standard pins : SCK, SDA, WS. Other MCU may have 5 pins)
	int NbIOPins;		//!< Total number of I/O pins
	I2SMODE Mode;		//!< Master/Slave mode
	I2SFMT Format;		//!< Format I2S/aligned-left/aligned-right
	I2SSWIDTH SampleWidth;	//!< Sample width 8/16/24 bits
	I2SCHAN Chan;		//! Channel stereo/left/right
	uint32_t Freq;		//! sampling frequency in Hz
	int MaxRetry;		//!< Max number of retry
	bool bDmaEn;		//!< true - Use DMA mode
	bool bIntEn;		//!< true - Interrupt enable
	int IntPrio;		//!< Interrupt priority
	DEVINTRF_EVTCB EvtCB;	//!< Interrupt based event callback function pointer. Must be set to NULL if not used
	uint8_t *pRxFifoMem;
	int RxFifoMemSize;
	uint8_t *pTxFifoMem;
	int TxFifoMemSize;
} I2SCFG;

/// Device driver data require by low level functions
typedef struct __I2S_Device {
	I2SMODE	Mode;		//!< Operating mode master/slave
	I2SFMT Format;		//!< Format
	uint8_t SampleWidth;//!< Sample width in bytes
	uint32_t Freq;		//! sampling frequency in mHz
	I2SCHAN Chan;		//! Channel stereo/left/right
	uint32_t MClkFreq;	//!< Master clock frequency in Hz
	DEVINTRF DevIntrf;	//!< Device interface instance
	const IOPINCFG *pIOPinMap;	//!< Define I/O pins used by I2S (standard pins : SCK, SDA, WS. Other MCU may have 5 pins)
	int NbIOPins;			//!< Total number of I/O pins
	HCFIFO hRxFifo;
	HCFIFO hTxFifo;
	DEVINTRF_EVTCB EvtCB;	//!< Interrupt based event callback function pointer. Must be set to NULL if not used
} I2SDEV;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Initialize I2S hardware interface
 *
 * This is a required implementation.
 *
 * @param	pDev : Pointer to device private data to be initialized
 * @param	pCfg : Pointer to configuration data
 *
 * @return	true - Initialization success
 */
bool I2SInit(I2SDEV * const pDev, const I2SCFG * const pCfg);

static inline void I2SEnable(I2SDEV *const pDev) { DeviceIntrfEnable(&pDev->DevIntrf); }
static inline void I2SDisable(I2SDEV *const pDev) { DeviceIntrfDisable(&pDev->DevIntrf); }

#ifdef __cplusplus
}

class I2S : public DeviceIntrf {
public:
	bool Init(const I2SCFG &Cfg) { return I2SInit(&vDevData, &Cfg); }
	operator DEVINTRF * const () { return &vDevData.DevIntrf; }
	operator I2SDEV& () { return vDevData; }
	operator I2SDEV* const () { return &vDevData; }
	uint32_t Rate(uint32_t RateHz) { return vDevData.DevIntrf.SetRate(&vDevData.DevIntrf, RateHz); }
	uint32_t Rate(void) { return vDevData.DevIntrf.GetRate(&vDevData.DevIntrf); }	// Get rate in Hz
	void Enable(void) { DeviceIntrfEnable(&vDevData.DevIntrf); }
	void Disable(void) { DeviceIntrfDisable(&vDevData.DevIntrf); }

	// DevCs is the ordinal starting from 0 of device connected to the SPI bus.
	// It is translated to CS index in the I/O pin map
	virtual bool StartRx(uint32_t DevAddr) {
		return DeviceIntrfStartRx(&vDevData.DevIntrf, DevAddr);
	}
	// Receive Data only, no Start/Stop condition
	virtual int RxData(uint8_t *pBuff, int BuffLen) {
		return DeviceIntrfRxData(&vDevData.DevIntrf, pBuff, BuffLen);
	}
	virtual void StopRx(void) { DeviceIntrfStopRx(&vDevData.DevIntrf); }
	// DevAddr is the ordinal starting from 0 of device connected to the SPI bus.
	// It is translated to CS index in the I/O pin map
	virtual bool StartTx(uint32_t DevAddr) {
		return DeviceIntrfStartTx(&vDevData.DevIntrf, DevAddr);
	}
	// Send Data only, no Start/Stop condition
	virtual int TxData(uint8_t *pData, int DataLen) {
		return DeviceIntrfTxData(&vDevData.DevIntrf, pData, DataLen);
	}
	virtual void StopTx(void) { DeviceIntrfStopTx(&vDevData.DevIntrf); }

protected:
private:
	I2SDEV vDevData;
};

#endif // __cplusplus

#endif // __I2S_H__
