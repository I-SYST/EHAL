/**-------------------------------------------------------------------------
@file	spi.h

@brief	Generic SPI (Serial Peripheral Interface) driver definitions.

Current implementation
 	 - Master mode
 	 - Polling

@author	Hoang Nguyen Hoan
@date	Nov. 20, 2011

@license

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
#ifndef __SPI_H__
#define __SPI_H__

#include <stdint.h>
#include <string.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "iopincfg.h"
#include "device_intrf.h"

/** @addtogroup device_intrf	Device Interface
  * @{
  */


// SPI Status code
typedef enum __SPI_Status {
	SPISTATUS_OK,
	SPISTATUS_TIMEOUT
} SPISTATUS;

typedef enum __SPI_Mode {
	SPIMODE_MASTER,				//!< SPI master
	SPIMODE_SLAVE				//!< SPI slave
} SPIMODE;

typedef enum __SPI_Clk_Polarity {
	SPICLKPOL_HIGH,				//!< Clock polarity active high : CPOL=0 idle low
	SPICLKPOL_LOW,				//!< Clock polarity active low : CPOL=1, idle high
} SPICLKPOL;

typedef enum __SPI_Data_Phase {
	SPIDATAPHASE_FIRST_CLK,		//!< Data phase starts on first clock transition leading : CPHA=0
	SPIDATAPHASE_SECOND_CLK		//!< Data phase starts on 2nd clock transition trailing : CPHA=1
} SPIDATAPHASE;

typedef enum __SPI_Data_Bit_Order {
	SPIDATABIT_MSB,				//!< Most significant bit first
	SPIDATABIT_LSB				//!< Least significant bit first
} SPIDATABIT;

typedef enum __SPI_Chip_Select {
	SPICSEL_AUTO,	//!< Select control internally by hardware or driver internally
	SPICSEL_MAN,	//!< Select control externally by application
	SPICSEL_DRIVER	//!< For driver's internal use only, do not set this in the config.
} SPICSEL;

// SPI physical interface
typedef enum __SPI_Phy {
	SPIPHY_NORMAL,				//!< Standard single SPI 4 wires CLK, MOSI, MISO, CS
	SPIPHY_4WIRE = SPIPHY_NORMAL,
	SPIPHY_3WIRE,				//!< 3 wires MISO/MOSI mux
	SPIPHY_DUAL,				//!< Dual SPI D0, D1 or used
	SPIPHY_QUAD_SDR,			//!< QSPI, single data rate
	SPIPHY_QUAD_DDR,			//!< QSPI, dual data rate
} SPIPHY;

#define SPI_MAX_RETRY			5

#define SPI_SLAVEMODE_MAX_DEV	4	//!< Max number of device (CS) supported in slave mode
 	 	 	 	 	 	 	 	 	//!< the real implementation may support less depending on hardware

/// SPI pins indexes
#define SPI_SCK_IOPIN_IDX		0
#define SPI_MISO_IOPIN_IDX		1
#define SPI_MOSI_IOPIN_IDX		2
#define SPI_CS_IOPIN_IDX		3	//!< Starting index for SPI chip select. This can
									//!< grow to allows multiple devices on same SPI.

/// Quad SPI pins indexes
#define QSPI_SCK_IOPIN_IDX		0
#define QSPI_D0_IOPIN_IDX		1
#define QSPI_D1_IOPIN_IDX		2
#define QSPI_D2_IOPIN_IDX		3
#define QSPI_D3_IOPIN_IDX		4
#define QSPI_CS_IOPIN_IDX		5	//!< Starting index for SPI chip select. This can
									//!< grow to allows multiple devices on same SPI.

#pragma pack(push, 4)

/// Configuration data used to initialize device
typedef struct __SPI_Config {
	int DevNo;				//!< SPI interface number identify by chip select (CS0, CS1,..,CSn)
	SPIPHY Phy;				//!< SPI physical interface type (standard, 3 wire, quad,..)
	SPIMODE Mode;			//!< Master/Slave mode
	const IOPINCFG *pIOPinMap;	//!< Define I/O pins used by SPI (including CS array)
	int NbIOPins;			//!< Total number of I/O pins
	int Rate;				//!< Speed in Hz
	uint32_t DataSize; 		//!< Data Size 4-16 bits
	int MaxRetry;			//!< Max number of retry
	SPIDATABIT BitOrder;	//!< Data bit ordering
	SPIDATAPHASE DataPhase;	//!< Data Out Phase.
	SPICLKPOL ClkPol;		//!< Clock Out Polarity.
	SPICSEL ChipSel;		//!< Chip select mode
	bool bDmaEn;			//!< true - Use DMA mode only on supported devices
	bool bIntEn;			//!< Interrupt enable
	int IntPrio;			//!< Interrupt priority
	DEVINTRF_EVTCB EvtCB;	//!< Event callback
} SPICFG;

typedef struct __QSPI_Cmd_Setup {
	uint8_t Cmd;
	uint8_t CmdMode;
	uint32_t Addr;
	uint8_t AddrLen;
	uint8_t AddrMode;
	uint32_t DataLen;
	uint8_t DataMode;
} QSPI_CMD_SETUP;

/// Device driver data require by low level functions
typedef struct __SPI_Device {
	SPICFG Cfg;				//!< Config data
	DEVINTRF DevIntrf;		//!< device interface implementation
	int	FirstRdData;		//!< This is to keep the first dummy read data of SPI
							//!< there are devices that may return a status code through this
	int	CurDevCs;			//!< Current active device CS
	uint8_t *pRxBuff[SPI_SLAVEMODE_MAX_DEV];//!< Pointer to slave mode rx buffer
	int RxBuffLen[SPI_SLAVEMODE_MAX_DEV];	//!< Rx buffer length in bytes
	uint8_t *pTxData[SPI_SLAVEMODE_MAX_DEV];//!< Pointer to slave mode tx data
	int TxDataLen[SPI_SLAVEMODE_MAX_DEV];	//!< Tx data length in bytes
} SPIDEV;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif	// __cplusplus

// Require implementations
bool SPIInit(SPIDEV * const pDev, const SPICFG *pCfgData);

static inline int SPIGetRate(SPIDEV * const pDev) { return pDev->DevIntrf.GetRate(&pDev->DevIntrf); }
static inline int SPISetRate(SPIDEV * const pDev, int Rate) {
	return pDev->DevIntrf.SetRate(&pDev->DevIntrf, Rate);
}
static inline void SPIEnable(SPIDEV * const pDev) { DeviceIntrfEnable(&pDev->DevIntrf); }
static inline void SPIDisable(SPIDEV * const pDev) { DeviceIntrfDisable(&pDev->DevIntrf); }
static inline int SPIRx(SPIDEV * const pDev, int DevCs, uint8_t *pBuff, int Bufflen) {
	return DeviceIntrfRx(&pDev->DevIntrf, DevCs, pBuff, Bufflen);
}
static inline int SPITx(SPIDEV * const pDev, int DevCs, uint8_t *pData, int DataLen) {
	return DeviceIntrfTx(&pDev->DevIntrf, DevCs, pData, DataLen);
}
static inline bool SPIStartRx(SPIDEV * const pDev, int DevAddr) {
	return DeviceIntrfStartRx(&pDev->DevIntrf, DevAddr);
}
static inline int SPIRxData(SPIDEV * const pDev, uint8_t *pBuff, int Bufflen) {
	return DeviceIntrfRxData(&pDev->DevIntrf, pBuff, Bufflen);
}
static inline void SPIStopRx(SPIDEV * const pDev) { DeviceIntrfStopRx(&pDev->DevIntrf); }
static inline bool SPIStartTx(SPIDEV * const pDev, int DevAddr) {
	return DeviceIntrfStartTx(&pDev->DevIntrf, DevAddr);
}
static inline int SPITxData(SPIDEV * const pDev, uint8_t *pData, int Datalen) {
	return DeviceIntrfTxData(&pDev->DevIntrf, pData, Datalen);
}
static inline void SPIStopTx(SPIDEV * const pDev) { DeviceIntrfStopTx(&pDev->DevIntrf); }

/**
 * @brief	Get current physical interface type
 */
static inline SPIPHY SPIGetPhy(SPIDEV * const pDev) { return pDev->Cfg.Phy; }

/**
 * @brief	Change SPI physical interface type
 *
 * This function allows dynamically switching between 3WIRE & NORMAL mode.
 * It is useful when a 3wire devices and standard devices are sharing the same SPI bus
 *
 * @param	pDev : Device Interface Handle
 * @param	Phy	 : New SPI physical interface type
 */
SPIPHY SPISetPhy(SPIDEV * const pDev, SPIPHY Phy);

/**
 * @brief	Set Quad SPI Flash size
 *
 * This function is available only and require for Quad SPI
 *
 * @param	pDev : Pointer SPI driver data initialized by SPIInit function
 * @param	Size : Flash memory size in KBytes
 */
void QuadSPISetMemSize(SPIDEV * const pDev, uint32_t Size);

/**
 * @brief	Configure and send command on Quad SPI interface
 *
 * This is only available and require for Quad SPI interface. Quad SPI is mainly used
 * for Flash memory
 *
 * @param	pDev : SPI device handle
 * @param	Cmd : Flash command code
 * @param	Addr : Address offset in flash memory to access. -1 if not used
 * @param	DataLen : Lenght of data in bytes to transfer
 * @param	DummyCycle : Number of dummy clock cycle
 *
 * @return	true - successful
 */
bool QuadSPISendCmd(SPIDEV * const pDev, uint8_t Cmd, uint32_t Addr, uint8_t AddrLen, uint32_t DataLen, uint8_t DummyCycle);

/**
 * @brief	Set SPI slave data for read command.
 *
 * This function sets internal pointer to the location of data to be returned to SPI master upon
 * receiving read command.
 *
 * @param	pDev	: Pointer SPI driver data initialized by SPIInit function
 * @param	SlaveIdx: Slave address index to assign the buffer
 * @param	pBuff	: Pointer to buffer to receive data from master
 * @param	BuffLen	: Total buffer length in bytes
 *
 * @return	None
 */
void SPISetSlaveRxBuffer(SPIDEV * const pDev, int SlaveIdx, uint8_t * const pBuff, int BuffLen);

/**
 * @brief	Set I2C slave buff for write command.
 *
 * This function sets internal pointer to the location of buffer to data from SPI master upon
 * receiving write command.
 *
 * @param	pDev	: Pointer I2C driver data initialized be I2CInit function
 * @param	SlaveIdx: Slave address index to assign the data buffer
 * @param	pData	: Pointer to data buffer to send to master
 * @param	DataLen	: Total data length in bytes
 *
 * @return	None
 */
void SPISetSlaveTxData(SPIDEV * const pDev, int SlaveIdx, uint8_t * const pData, int DataLen);


#ifdef __cplusplus
}

/// C++ SPI class wrapper
class SPI : public DeviceIntrf {
public:
	SPI() {
		memset((void*)&vDevData, 0, (int)sizeof(vDevData));
	}

	virtual ~SPI() {
	}

	SPI(SPI&);	// Copy ctor not allowed

	bool Init(const SPICFG &CfgData) { return SPIInit(&vDevData, &CfgData); }

	operator DEVINTRF * const () { return &vDevData.DevIntrf; }
	operator SPIDEV& () { return vDevData; };			// Get config data
	operator SPIDEV * const () { return &vDevData; };	// Get pointer to device data
	uint32_t Rate(uint32_t RateHz) { return vDevData.DevIntrf.SetRate(&vDevData.DevIntrf, RateHz); }
	uint32_t Rate(void) { return vDevData.DevIntrf.GetRate(&vDevData.DevIntrf); }	// Get rate in Hz
	void Enable(void) { DeviceIntrfEnable(&vDevData.DevIntrf); }
	void Disable(void) { DeviceIntrfDisable(&vDevData.DevIntrf); }

	// DevCs is the ordinal starting from 0 of device connected to the SPI bus.
	// It is translated to CS index in the I/O pin map
	virtual bool StartRx(uint32_t DevCs) {
		return DeviceIntrfStartRx(&vDevData.DevIntrf, DevCs);
	}
	// Receive Data only, no Start/Stop condition
	virtual int RxData(uint8_t *pBuff, int BuffLen) {
		return DeviceIntrfRxData(&vDevData.DevIntrf, pBuff, BuffLen);
	}
	virtual void StopRx(void) { DeviceIntrfStopRx(&vDevData.DevIntrf); }
	// DevAddr is the ordinal starting from 0 of device connected to the SPI bus.
	// It is translated to CS index in the I/O pin map
	virtual bool StartTx(uint32_t DevCs) {
		return DeviceIntrfStartTx(&vDevData.DevIntrf, DevCs);
	}
	// Send Data only, no Start/Stop condition
	virtual int TxData(uint8_t *pData, int DataLen) {
		return DeviceIntrfTxData(&vDevData.DevIntrf, pData, DataLen);
	}
	virtual void StopTx(void) { DeviceIntrfStopTx(&vDevData.DevIntrf); }
	int GetFirstRead(void) {return vDevData.FirstRdData;}

	/**
	 * @brief	Set SPI slave rx buffer to receive data from master.
	 *
	 * This function sets internal pointer to the location of data to be returned to SPI master upon
	 * receiving read command.
	 *
	 * @param	SlaveIdx: Slave address index to assign the buffer
	 * @param	pBuff	: Pointer to buffer to receive data from master
	 * @param	BuffLen	: Total buffer length in bytes
	 *
	 * @return	None
	 */
	virtual void SetSlaveRxBuffer(int SlaveIdx, uint8_t * const pBuff, int BuffLen) {
		SPISetSlaveRxBuffer(&vDevData, SlaveIdx, pBuff, BuffLen);
	}

	/**
	 * @brief	Set I2C slave tx data buffer to se to master.
	 *
	 * This function sets internal pointer to the location of buffer to data from SPI master upon
	 * receiving write command.
	 *
	 * @param	SlaveIdx: Slave address index to assign the data buffer
	 * @param	pData	: Pointer to data buffer to send to master
	 * @param	DataLen	: Total data length in bytes
	 *
	 * @return	None
	 */
	virtual void SetSlaveTxData(int SlaveIdx, uint8_t * const pData, int DataLen) {
		SPISetSlaveTxData(&vDevData, SlaveIdx, pData, DataLen);
	}

	virtual SPIPHY Phy() { return vDevData.Cfg.Phy; }
	virtual SPIPHY Phy(SPIPHY Phy) {
		vDevData.Cfg.Phy = SPISetPhy(&vDevData, Phy);
		return vDevData.Cfg.Phy;
	}

private:
	SPIDEV vDevData;
};

#endif	// __cplusplus

/** @} end group device_intrf */

#endif	// __SPI_H__
