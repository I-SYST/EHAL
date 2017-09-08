/*--------------------------------------------------------------------------
File   : esb_intrf.h

Author : Hoang Nguyen Hoan          				June 13, 2017

Desc   : Implementation allow the creation of generic device interface for
		 ESB (Enhanced Shock Burst)

Copyright (c) 2017, I-SYST inc., all rights reserved

Contributor

Copyright (c) 2017, Motsai, all rights reserved

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
#ifndef __ESB_INTRF_H__
#define __ESB_INTRF_H__

#include "nrf_esb.h"

#include "device_intrf.h"
#include "cfifo.h"

#define ESBINTRF_CFIFO_MEMSIZE(NbBlk)      CFIFO_TOTAL_MEMSIZE(NbBlk, sizeof(nrf_esb_payload_t))

#pragma pack(push, 4)
typedef struct __EsbDeviceInterfConfig {
    uint8_t BaseAddr0[4];
    uint8_t BaseAddr1[4];
    uint8_t PipePrefix[8];
    nrf_esb_protocol_t  Protocol;
    nrf_esb_mode_t      Mode;
    nrf_esb_tx_power_t  TxPower;
    int     Rate;
    int     PacketSize;     // data packet size
    int     NbRetry;
    int     RxFifoMemSize;  // Total memory size for CFIFO
    uint8_t *pRxFifoMem;    // Pointer to memory to be used by CFIFO
    int     TxFifoMemSize;  // Total memory size for CFIFO
    uint8_t *pTxFifoMem;    // Pointer to memory to be used by CFIFO
    DEVINTRF_EVTCB EvtCB;   // Event callback
} ESBINTRF_CFG;

// BLE interf instance data
typedef struct __BleDeviceInterf {
    DEVINTRF    DevIntrf;   // Base Device Interface
    int         PacketSize; // BLE packet size
    nrf_esb_bitrate_t Rate;
    HCFIFO      hRxFifo;
    HCFIFO      hTxFifo;
    nrf_esb_config_t EsbCfg;
} ESBINTRF;
#pragma pack(pop)

class EsbIntrf : public DeviceIntrf {
public:
    EsbIntrf();
    virtual ~EsbIntrf();

    bool Init(const ESBINTRF_CFG &Cfg);

    operator DEVINTRF* () { return &vEsbIntrf.DevIntrf; }   // No C implementation, just return NULL
    // Set data rate in bits/sec (Hz)
    int Rate(int DataRate) { return DeviceIntrfSetRate(&vEsbIntrf.DevIntrf, DataRate); }
    // Get current data rate in bits/sec (Hz)
    int Rate(void) { return DeviceIntrfGetRate(&vEsbIntrf.DevIntrf); }
    // Disable device for power reduction, re-enable with Enable() without
    // full init
    void Disable(void) { DeviceIntrfDisable(&vEsbIntrf.DevIntrf); }
    // Enable device
     void Enable(void) { DeviceIntrfEnable(&vEsbIntrf.DevIntrf); }

    // Initiate receive
    virtual bool StartRx(int DevAddr) { return DeviceIntrfStartRx(&vEsbIntrf.DevIntrf, DevAddr); }
    // Receive Data only, no Start/Stop condition
    virtual int RxData(uint8_t *pBuff, int BuffLen) {
        return DeviceIntrfRxData(&vEsbIntrf.DevIntrf, pBuff, BuffLen);
    }
    // Stop receive
    // BEWARE !!!!!
    // This functions MUST ONLY be called if StartRx returns true.
    virtual void StopRx(void) { DeviceIntrfStopRx(&vEsbIntrf.DevIntrf); }
    // Initiate transmit
    virtual bool StartTx(int DevAddr) {
        return DeviceIntrfStartTx(&vEsbIntrf.DevIntrf, DevAddr);
    }
    // Transmit Data only, no Start/Stop condition
    virtual int TxData(uint8_t *pData, int DataLen) {
        return DeviceIntrfTxData(&vEsbIntrf.DevIntrf, pData, DataLen);
    }
    // Stop transmit
    // BEWARE !!!!!
    // This functions MUST ONLY be called if StartTx returns true.
    virtual void StopTx(void) { DeviceIntrfStopTx(&vEsbIntrf.DevIntrf); }

private:
    ESBINTRF vEsbIntrf;
};

#endif // __ESB_INTRF_H__

