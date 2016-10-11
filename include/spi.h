/*--------------------------------------------------------------------------
File   : spi.h

Author : Hoang Nguyen Hoan          Nov. 20, 2011

Desc   : Generic SPI definitions
		 Current implementation
		 	 Master mode
		 	 Polling

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

----------------------------------------------------------------------------
Modified by         Date            Description
Hoan				Feb. 20, 2015	New EHAL
----------------------------------------------------------------------------*/
#ifndef __SPI_H__
#define __SPI_H__

#include <stdint.h>
#include <string.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "iopincfg.h"
#include "serialintrf.h"



// SPI Status code
typedef enum _SPI_Status {
	SPISTATUS_OK
} SPISTATUS;

typedef enum _SPI_Mode {
	SPIMODE_MASTER,
	SPIMODE_SLAVE
} SPIMODE;

typedef enum _SPI_Clk_Polarity {
	SPICLKPOL_HIGH,
	SPICLKPOL_LOW
} SPICLKPOL;

typedef enum _SPI_Data_Phase {
	SPIDATAPHASE_FIRST_CLK,		// Data phase starts on first clock
	SPIDATAPHASE_SECOND_CLK		// Data phase starts on 2nd clock
} SPIDATAPHASE;

typedef enum _SPI_Data_Bit_Order {
	SPIDATABIT_MSB,				// Most significant bit first
	SPIDATABIT_LSB				// Least significant bit first
} SPIDATABIT;

#define SPI_MAX_RETRY			5
#define SPI_MAX_NB_IOPIN		4
#define SPI_SCK_IOPIN_IDX		0
#define SPI_MISO_IOPIN_IDX		1
#define SPI_MOSI_IOPIN_IDX		2
#define SPI_SS_IOPIN_IDX		3

#pragma pack(push, 4)

// Configuration data used to initialize device
typedef struct _SPI_Config {
	int DevNo;				// SPI interface number
	SPIMODE Mode;			// Master/Slave mode
	IOPINCFG IOPinMap[SPI_MAX_NB_IOPIN];	// Define I/O pins used by SPI
	int Rate;				// Speed in Hz
	uint32_t DataSize; 		// Data Size 4-16 bits
	int SlaveAddr;			// slave address used in slave mode only
	int MaxRetry;			// Max number of retry
	SPIDATABIT BitOrder;	// Data bit ordering
	SPIDATAPHASE DataPhase;	// Data Out Phase.
	SPICLKPOL ClkPol;		// Clock Out Polarity.
	bool bManSel;        	// Set to true for manual select
	int IntPrio;			// Interrupt priority
	SERINTRFEVCB EvtCB;		// Event callback
} SPICFG;

// Device driver data require by low level fonctions
typedef struct {
	SPICFG 	Cfg;			// Config data
	SERINTRFDEV	SerIntrf;	// device interface implementation
	int		FirstRdData;	// This is to keep the first dummy read data of SPI
							// there are devices that may return a status code through this
} SPIDEV;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif	// __cplusplus

// Require impplementations
bool SPIInit(SPIDEV *pDev, const SPICFG *pCfgData);

static inline int SPIGetRate(SPIDEV *pDev) { return pDev->SerIntrf.GetRate(&pDev->SerIntrf); }
static inline int SPISetRate(SPIDEV *pDev, int Rate) { return pDev->SerIntrf.SetRate(&pDev->SerIntrf, Rate); }
static inline int SPIRx(SPIDEV *pDev, uint8_t *pBuff, int Bufflen) {
	int len = 0;

	pDev->SerIntrf.StartRx(&pDev->SerIntrf, 0);
	len = pDev->SerIntrf.RxData(&pDev->SerIntrf, pBuff, Bufflen);
	pDev->SerIntrf.StopRx(&pDev->SerIntrf);

	return len;
}
static inline int SPITx(SPIDEV *pDev, uint8_t *pData, int Datalen) {
	int len = 0;

    pDev->SerIntrf.StartTx(&pDev->SerIntrf, 0);
    len = pDev->SerIntrf.TxData(&pDev->SerIntrf, pData, Datalen);
    pDev->SerIntrf.StopTx(&pDev->SerIntrf);

    return len;
}


#ifdef __cplusplus
}

// C++ class wrapper

class SPI : public SerialIntrf {
public:
	SPI() {
		memset((void*)&vDevData, 0, (int)sizeof(vDevData));
	}

	virtual ~SPI() {
		Disable();
	}

	SPI(SPI&);	// Copy ctor not allowed

	bool Init(const SPICFG &CfgData) { return SPIInit(&vDevData, &CfgData); }

	operator SPIDEV& () { return vDevData; };	// Get config data
	int Rate(int RateHz) { return SerialIntrfSetRate(&vDevData.SerIntrf, RateHz); }
	int Rate(void) { return SPIGetRate(&vDevData); };	// Get rate in Hz
	void Enable(void) { SerialIntrfEnable(&vDevData.SerIntrf); }
	void Disable(void) { SerialIntrfDisable(&vDevData.SerIntrf); }
	virtual bool StartRx(int DevAddr) {
		return SerialIntrfStartRx(&vDevData.SerIntrf, DevAddr);
	}
	// Receive Data only, no Start/Stop condition
	virtual int RxData(uint8_t *pBuff, int BuffLen) {
		return SerialIntrfRxData(&vDevData.SerIntrf, pBuff, BuffLen);
	}
	virtual void StopRx(void) { SerialIntrfStopRx(&vDevData.SerIntrf); }
	virtual bool StartTx(int DevAddr) {
		return SerialIntrfStartTx(&vDevData.SerIntrf, DevAddr);
	}
	// Send Data only, no Start/Stop condition
	virtual int TxData(uint8_t *pData, int DataLen) {
		return SerialIntrfTxData(&vDevData.SerIntrf, pData, DataLen);
	}
	virtual void StopTx(void) { SerialIntrfStopTx(&vDevData.SerIntrf); }
	int GetFirstRead(void) {return vDevData.FirstRdData;}

private:
	SPIDEV vDevData;
};

#endif	// __cplusplus

#endif	// __SPI_H__
