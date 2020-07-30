/**-------------------------------------------------------------------------
@file	i2c.h

@brief	Generic I2C driver definitions

Current implementation
 	 - Master mode
 	 - Slave mode
 	 - Polling
 	 - Interrupt


@author	Hoang Nguyen Hoan
@date	Nov. 20, 2011


Copyright (c) 2011, I-SYST inc., all rights reserved

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
#ifndef __I2C_H__
#define __I2C_H__

#include <stdint.h>
#include <string.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "device_intrf.h"
#include "iopincfg.h"

/** @addtogroup device_intrf	Device Interface
  * @{
  */

/// Minimum timing
#define I2C_SCL_STD_MODE_MAX_SPEED			100		// KHz
#define I2C_SCL_TLOW_STD_MODE_MIN			4700	// ns
#define I2C_SCL_THIGH_STD_MODE_MIN			4000	// ns

#define I2C_SCL_FAST_MODE_MAX_SPEED			400		// KHz
#define I2C_SCL_TLOW_FAST_MODE_MIN			1300	// ns
#define I2C_SCL_THIGH_FAST_MODE_MIN			600		// ns

#define I2C_SCL_FAST_MODE_PLUS_MAX_SPEED	1000	// KHz
#define I2C_SCL_TLOW_FAST_MODE_PLUS_MIN		500		// ns
#define I2C_SCL_THIGH_FAST_MODE_PLUS_MIN	260		// ns

/// I2C Status code
typedef enum __I2C_Status {
	I2CSTATUS_START_COND = 8,		//!< Start condition transmitted
	I2CSTATUS_RESTART_COND = 0x10,	//!< Start condition re-transmitted
	I2CSTATUS_SLAW_ACK = 0x18,		//!< SLA+W has been transmitted; ACK has been received
	I2CSTATUS_SLAW_NACK = 0x20,		//!< SLA+W has been transmitted; NO ACK has been received
	I2CSTATUS_M_TXDAT_ACK = 0x28,	//!< Data byte in I2DAT has been transmitted; ACK has been received
	I2CSTATUS_M_TXDAT_NACK = 0x30,	//!< Data byte in I2DAT has been transmitted; NO ACK has been received
	I2CSTATUS_ARB_LOST = 0x38,		//!< Arbitration lost in SLA+R/W or Data bytes
	I2CSTATUS_SLAR_ACK = 0x40,		//!< SLA+R has been transmitted; ACK has been received
	I2CSTATUS_SLAR_NACK = 0x48,		//!< SLA+R has been transmitted; NO ACK has been received
	I2CSTATUS_RXDATA_ACK = 0x50,	//!< Data byte has been received; ACK has been returned
	I2CSTATUS_RXDATA_NACK = 0x58,	//!< ata byte has been received; NO ACK has been returned
	I2CSTATUS_OWNSLAW_ACK = 0x60,	//!< Own SLA+W has been received; ACK has been returned
	I2CSTATUS_ARB_LOST_OWNSLAW_NACK = 0x68,	//!< Arbitration lost in SLA+R/W as master; Own SLA+W has been received, ACK returned
	I2CSTATUS_GENCALL_ACK = 0x70,	//!< General Call address (0x00) has been received; ACK has been returned
	I2CSTATUS_ARB_LOST_GENCALL_NACK = 0x78,	//!< Arbitration lost in SLA+R/W as master; General Call address has been received, ACK has been returned
	I2CSTATUS_SLA_OWN_DATA_ACK = 0x80,	//!< Previously addressed with own SLA address; DATA has been received; ACK has been returned
	I2CSTATUS_SLA_OWN_DATA_NACK = 0x88,	//!< Previously addressed with own SLA address; DATA has been received; NO ACK has been returned
	I2CSTATUS_GENCALL_DATA_ACK = 0x90,	//!< Previously addressed with General Call; DATA byte has been received; ACK has been returned
	I2CSTATUS_GENCALL_DATA_NACK = 0x98,	//!< Previously addressed with General Call; DATA byte has been received; NO ACK has been returned
	I2CSTATUS_STOP_COND = 0xA0,		//!< A STOP condition or repeated START condition has been received while still addressed as Slave Receiver or Slave Transmitter
	I2CSTATUS_OWN_SLAR_ACK = 0xA8,	//!< Own SLA+R has been received; ACK has been returned
	I2CSTATUS_ARB_LOST_OWN_SLAR_ACK = 0xB0,	//!< Arbitration lost in SLA+R/W as master; Own SLA+R has been received, ACK has been returned
	I2CSTATUS_S_TXDAT_ACK = 0xB8,	//!< Data byte in I2DAT has been transmitted; ACK has been received
	I2CSTATUS_S_TXDAT_NACK = 0xC0,	//!< Data byte in I2DAT has been transmitted; NOT ACK has been received
	I2CSTATUS_S_LAST_TXDAT_ACK = 0xC8,	//!< Last data byte in I2DAT has been transmitted (AA = 0); ACK has been received
} I2CSTATUS;

typedef enum __I2C_Mode {
	I2CMODE_MASTER,
	I2CMODE_SLAVE
} I2CMODE;


#define I2C_SLAVEMODE_MAX_ADDR		4	//!< Max number of response addresses in slave mode
	 									//!< the real implementation may support less depending on hardware

#define I2C_MAX_RETRY				5	//!< Max number of retries

#define I2C_MAX_NB_IOPIN			2	//!< Number of I/O pins needed by I2C

/// I/O pin map index
#define I2C_SDA_IOPIN_IDX			0	//!< SDA pin index
#define I2C_SCL_IOPIN_IDX			1	//!< SCL pin index

#pragma pack(push, 4)

/// Configuration data used to initialize device
typedef struct __I2C_Config {
	int DevNo;				//!< I2C interface number
	IOPINCFG Pins[I2C_MAX_NB_IOPIN];	//!< Define I/O pins used by I2C
	int Rate;				//!< Speed in Hz
	I2CMODE Mode;			//!< Master/Slave mode
	int MaxRetry;			//!< Max number of retry
	int NbSlaveAddr;		//!< Number of slave mode address configured
	uint8_t SlaveAddr[I2C_SLAVEMODE_MAX_ADDR];	//!< I2C slave address used in slave mode only
	bool bDmaEn;			//!< true - Use DMA mode only on supported devices
	bool bIntEn;			//!< true - Interrupt enable
	int	IntPrio;			//!< Interrupt priority.  Value is implementation specific
	DEVINTRF_EVTCB EvtCB;	//!< Interrupt based event callback function pointer. Must be set to NULL if not used
	bool bSmBus;			//!< Enable SMBUS support
} I2CCFG;

/// Device driver data require by low level functions
typedef struct {
	I2CMODE Mode;			//!< Operating mode Master/Slave
	int 	Rate;			//!< Speed in Hz
	int 	NbSlaveAddr;	//!< Number of slave mode address configured
	uint8_t	SlaveAddr[I2C_SLAVEMODE_MAX_ADDR];	//!< I2C slave address used in slave mode only
	DEVINTRF DevIntrf;		//!< I2C device interface implementation
	IOPINCFG Pins[I2C_MAX_NB_IOPIN];			//!< Define I/O pins used by I2C
	uint8_t *pRRData[I2C_SLAVEMODE_MAX_ADDR];	//!< Pointer to data buffer to return upon receiving read request
	int RRDataLen[I2C_SLAVEMODE_MAX_ADDR];		//!< Read request data length in bytes
	uint8_t *pTRBuff[I2C_SLAVEMODE_MAX_ADDR];	//!< Pointer to buffer to receive data upon receiving write request
	int TRBuffLen[I2C_SLAVEMODE_MAX_ADDR];		//!< Write request buffer length in bytes
	bool bSmBus;			//!< SMBUS support
} I2CDEV;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif	// __cplusplus

/**
 * @brief - I2C initialization.
 *
 * The main initialization of the i2c engine.  This must be implemented per architecture.
 *
 * @param	pDev 		: Pointer to be filed by this init function with implementation specific data
 * @param	pCfgData 	: Pointer to I2C configuration
 *
 * @return
 * 			- true	: Success
 * 			- false	: Failed
 */
bool I2CInit(I2CDEV * const pDev, const I2CCFG *pCfgData);
void I2CBusReset(I2CDEV * const pDev);
static inline int I2CGetRate(I2CDEV * const pDev) { return pDev->DevIntrf.GetRate(&pDev->DevIntrf); }
static inline int I2CSetRate(I2CDEV * const pDev, int Rate) {
	return pDev->DevIntrf.SetRate(&pDev->DevIntrf, Rate);
}
static inline void I2CEnable(I2CDEV * const pDev) { DeviceIntrfEnable(&pDev->DevIntrf); }
static inline void I2CDisable(I2CDEV * const pDev) { DeviceIntrfDisable(&pDev->DevIntrf); }
static inline int I2CRx(I2CDEV * const pDev, int DevAddr, uint8_t *pBuff, int Bufflen) {
	return DeviceIntrfRx(&pDev->DevIntrf, DevAddr, pBuff, Bufflen);
}
static inline int I2CTx(I2CDEV * const pDev, int DevAddr, uint8_t *pData, int Datalen) {
	return DeviceIntrfTx(&pDev->DevIntrf, DevAddr, pData, Datalen);
}
static inline int I2CRead(I2CDEV * const pDev, int DevAddr, uint8_t *pAdCmd, int AdCmdLen,
        uint8_t *pRxBuff, int RxLen) {
	return DeviceIntrfRead(&pDev->DevIntrf, DevAddr, pAdCmd, AdCmdLen, pRxBuff, RxLen);
}
static inline int I2CWrite(I2CDEV * const pDev, int DevAddr, uint8_t *pAdCmd, int AdCmdLen,
        uint8_t *pTxData, int TxLen) {
	return DeviceIntrfWrite(&pDev->DevIntrf, DevAddr, pAdCmd, AdCmdLen, pTxData, TxLen);
}
static inline bool I2CStartRx(I2CDEV * const pDev, int DevAddr) {
	return DeviceIntrfStartRx(&pDev->DevIntrf, DevAddr);
}
static inline int I2CRxData(I2CDEV * const pDev, uint8_t *pBuff, int Bufflen) {
	return DeviceIntrfRxData(&pDev->DevIntrf, pBuff, Bufflen);
}
static inline void I2CStopRx(I2CDEV * const pDev) { DeviceIntrfStopRx(&pDev->DevIntrf); }
static inline bool I2CStartTx(I2CDEV * const pDev, int DevAddr) {
	return DeviceIntrfStartTx(&pDev->DevIntrf, DevAddr);
}
static inline int I2CTxData(I2CDEV * const pDev, uint8_t *pData, int Datalen) {
	return DeviceIntrfTxData(&pDev->DevIntrf, pData, Datalen);
}
static inline void I2CStopTx(I2CDEV * const pDev) { DeviceIntrfStopTx(&pDev->DevIntrf); }

/**
 * @brief	Set I2C slave data for read command.
 *
 * This function sets internal pointer to the location of data to be returned to I2C master upon
 * receiving read command.
 *
 * @param	pDev	: Pointer I2C driver data initialized be I2CInit function
 * @param	SlaveIdx: Slave address index to assign the data buffer
 * @param	pData	: Pointer to data buffer to send for read command
 * @param	DataLen	: Total data length in bytes
 *
 * @return	None
 */
void I2CSetReadRqstData(I2CDEV * const pDev, int SlaveIdx, uint8_t * const pData, int DataLen);

/**
 * @brief	Set I2C slave buff for write command.
 *
 * This function sets internal pointer to the location of buffer to data from I2C master upon
 * receiving write command.
 *
 * @param	pDev	: Pointer I2C driver data initialized be I2CInit function
 * @param	SlaveIdx: Slave address index to assign the data buffer
 * @param	pBuff	: Pointer to data buffer to receive for write command
 * @param	BuffLen	: Total data length in bytes
 *
 * @return	None
 */
void I2CSetWriteRqstBuffer(I2CDEV * const pDev, int SlaveIdx, uint8_t * const pBuff, int BuffLen);

#ifdef __cplusplus
}

/// Generic I2C base class
class I2C : public DeviceIntrf {
public:
	I2C() {
		memset((void*)&vDevData, 0, (int)sizeof(vDevData));
	}

	virtual ~I2C() {
		Disable();
	}

	I2C(I2C&);	// Copy ctor not allowed

	bool Init(const I2CCFG &CfgData) { return I2CInit(&vDevData, &CfgData); }
	operator DEVINTRF * const () { return &vDevData.DevIntrf; }
	operator I2CDEV& () { return vDevData; };	// Get config data
	uint32_t Rate(uint32_t RateHz) { return DeviceIntrfSetRate(&vDevData.DevIntrf, RateHz); }
	uint32_t Rate(void) { return vDevData.Rate; };	// Get rate in Hz
	void Enable(void) { DeviceIntrfEnable(&vDevData.DevIntrf); }
	void Disable(void) { DeviceIntrfDisable(&vDevData.DevIntrf); }
	virtual bool StartRx(uint32_t DevAddr) {
		return DeviceIntrfStartRx(&vDevData.DevIntrf, DevAddr);
	}
	// Receive Data only, no Start/Stop condition
	virtual int RxData(uint8_t *pBuff, int BuffLen) {
		return DeviceIntrfRxData(&vDevData.DevIntrf, pBuff, BuffLen);
	}
	virtual void StopRx(void) { DeviceIntrfStopRx(&vDevData.DevIntrf); }
	virtual bool StartTx(uint32_t DevAddr) {
		return DeviceIntrfStartTx(&vDevData.DevIntrf, DevAddr);
	}
	// Send Data only, no Start/Stop condition
	virtual int TxData(uint8_t *pData, int DataLen) {
		return DeviceIntrfTxData(&vDevData.DevIntrf, pData, DataLen);
	}
	virtual void StopTx(void) { DeviceIntrfStopTx(&vDevData.DevIntrf); }
	
	/**
	 * @brief	Set I2C slave data for read command.
	 *
	 * This function sets internal pointer to the location of data to be returned to I2C master upon
	 * receiving read command.
	 *
	 * @param	SlaveIdx: Slave address index to assign the data buffer
	 * @param	pData	: Pointer to data buffer to send for read command
	 * @param	DataLen	: Total data length in bytes
	 *
	 * @return	None
	 */
	virtual void SetReadRqstData(int SlaveIdx, uint8_t * const pData, int DataLen) {
		I2CSetReadRqstData(&vDevData, SlaveIdx, pData, DataLen);
	}

	/**
	 * @brief	Set I2C slave buff for write command.
	 *
	 * This function sets internal pointer to the location of buffer to data from I2C master upon
	 * receiving write command.
	 *
	 * @param	SlaveIdx: Slave address index to assign the data buffer
	 * @param	pBuff	: Pointer to data buffer to receive for write command
	 * @param	BuffLen	: Total data length in bytes
	 *
	 * @return	None
	 */
	virtual void SetWriteRqstBuffer(int SlaveIdx, uint8_t * const pBuff, int BuffLen) {
		I2CSetWriteRqstBuffer(&vDevData, SlaveIdx, pBuff, BuffLen);
	}

private:
	I2CDEV vDevData;
};

#endif	// __cplusplus

/** @} end group device_intrf */

#endif	// __I2C_H__
