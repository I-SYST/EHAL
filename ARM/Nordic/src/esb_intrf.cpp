/*--------------------------------------------------------------------------
File   : esb_intrf.cpp

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
#include "nrf_error.h"
#include "nrf_esb_error_codes.h"
#include "sdk_common.h"

#include "esb_intrf.h"


static uint8_t s_EsbRxFifoMem[ESBINTRF_CFIFO_MEMSIZE(2)];
static uint8_t s_EsbTxFifoMem[ESBINTRF_CFIFO_MEMSIZE(2)];

static EsbIntrf *s_pEsbDevice = NULL;
static volatile bool s_bEsbTxReady = true;

/**
 * @brief - Disable
 *      Turn off the interface.  If this is a physical interface, provide a
 * way to turn off for energy saving. Make sure the turn off procedure can
 * be turned back on without going through the full init sequence
 *
 * @param
 *      pDevIntrf : Pointer to an instance of the Device Interface
 *
 * @return None
 */
void EsbIntrfDisable(DEVINTRF *pDevIntrf)
{
    nrf_esb_disable();
}

/**
 * @brief - Enable
 *      Turn on the interface.
 *
 * @param
 *      pDevIntrf : Pointer to an instance of the Device Interface
 *
 * @return None
 */
void EsbIntrfEnable(DEVINTRF *pDevIntrf)
{
	ESBINTRF *dev = (ESBINTRF*)pDevIntrf->pDevData;

    uint32_t err_code = nrf_esb_init(&dev->EsbCfg);
//    VERIFY_SUCCESS(err_code);
/*
    err_code = nrf_esb_set_base_address_0(dev->BaseAddr0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(dev->BaseAddr1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(dev->PipePrefix, 8);
    VERIFY_SUCCESS(err_code);*/
}

/**
 * @brief - GetRate
 *      Get data rate of the interface in Hertz.  This is not a clock frequency
 * but rather the transfer frequency (number of transfers per second). It has meaning base on the
 * implementation as bits/sec or bytes/sec or whatever the case
 *
 * @param
 *      pDevIntrf : Pointer to an instance of the Device Interface
 *
 * @return Transfer rate per second
 */
int EsbIntrfGetRate(DEVINTRF *pDevIntrf)
{
    ESBINTRF *dev = (ESBINTRF*)pDevIntrf->pDevData;
    int rate = 0;

    switch (dev->Rate)
    {
        case NRF_ESB_BITRATE_250KBPS:
            rate = 250000;
            break;
        case NRF_ESB_BITRATE_1MBPS:
        case NRF_ESB_BITRATE_1MBPS_BLE:
            rate = 1000000;
            break;
        case NRF_ESB_BITRATE_2MBPS:
            rate = 2000000;
            break;
    }

    return rate;
}

/**
 * @brief - SetRate
 *      Set data rate of the interface in Hertz.  This is not a clock frequency
 * but rather the transfer frequency (number of transfers per second). It has meaning base on the
 * implementation as bits/sec or bytes/sec or whatever the case
 *
 * @param
 *      pDevIntrf : Pointer to an instance of the Device Interface
 *      Rate      : Data rate to be set in Hertz (transfer per second)
 *
 * @return  Actual transfer rate per second set.  It is the real capable rate
 *          closes to rate being requested.
 */
int EsbIntrfSetRate(DEVINTRF *pDevIntrf, int Rate)
{
    // Data rate can't be changed on the fly
    // just return current rate

    return pDevIntrf->GetRate(pDevIntrf);
}

/**
 * @brief - StartRx
 *      Prepare start condition to receive data with subsequence RxData.
 * This can be in case such as start condition for I2C or Chip Select for
 * SPI or precondition for DMA transfer or whatever requires it or not
 * This function must check & set the busy state for re-entrancy
 *
 * @param
 *      pDevIntrf : Pointer to an instance of the Device Interface
 *      DevAddr   : The device selection id scheme
 *
 * @return  true - Success
 *          false - failed.
 */
bool EsbIntrfStartRx(DEVINTRF *pDevIntrf, int DevAddr)
{
    return true;
}

/**
 * @brief - RxData : retrieve 1 packet of received data
 *      Receive data into pBuff passed in parameter.  Assuming StartRx was
 * called prior calling this function to get the actual data. BufferLen
 * to receive data must be at least 1 packet in size.  Otherwise remaining
 * bytes are dropped.
 *
 * @param
 *      pDevIntrf : Pointer to an instance of the Device Interface
 *      pBuff     : Pointer to memory area to receive data.
 *      BuffLen   : Length of buffer memory in bytes. Must be at least 1 packet
 *                  in size.  Otherwise remaining bytes are dropped.
 *
 * @return  Number of bytes read
 */
int EsbIntrfRxData(DEVINTRF *pDevIntrf, uint8_t *pBuff, int BuffLen)
{
    ESBINTRF *intrf = (ESBINTRF*)pDevIntrf->pDevData;
    nrf_esb_payload_t *payload;
    int cnt = 0;

    payload = (nrf_esb_payload_t *)CFifoGet(intrf->hRxFifo);
    if (payload != NULL)
    {
        cnt = min(BuffLen, payload->length);
        memcpy(pBuff, payload->data, cnt);
    }

    return cnt;
}

/**
 * @brief - StopRx
 *      Completion of read data phase. Do require post processing
 * after data has been received via RxData
 * This function must clear the busy state for re-entrancy
 *
 * @param
 *      pDevIntrf : Pointer to an instance of the Device Interface
 *
 * @return  None
 */
void EsbIntrfStopRx(DEVINTRF *pSerDev)
{
    // TODO:
}

/**
 * @brief - StartTx
 *      Prepare start condition to transfer data with subsequence TxData.
 * This can be in case such as start condition for I2C or Chip Select for
 * SPI or precondition for DMA transfer or whatever requires it or not
 * This function must check & set the busy state for re-entrancy
 *
 * @param
 *      pDevIntrf : Pointer to an instance of the Device Interface
 *      DevAddr   : The device selection id scheme
 *
 * @return  true - Success
 *          false - failed
 */
bool EsbIntrfStartTx(DEVINTRF *pDevIntrf, int DevAddr)
{
    return true;
}

/**
 * @brief - TxData
 *      Transfer data from pData passed in parameter.  Assuming StartTx was
 * called prior calling this function to send the actual data
 *
 * @param
 *      pDevIntrf : Pointer to an instance of the Device Interface
 *      pData   : Pointer to memory area of data to send.
 *      DataLen : Length of data memory in bytes
 *
 * @return  Number of bytes sent
 */
int EsbIntrfTxData(DEVINTRF *pDevIntrf, uint8_t *pData, int DataLen)
{
    ESBINTRF *intrf = (ESBINTRF*)pDevIntrf->pDevData;
    nrf_esb_payload_t *payload;
    int cnt = 0;

    while (DataLen > 0)
    {
        payload = (nrf_esb_payload_t *)CFifoPut(intrf->hTxFifo);
        if (payload == NULL)
            break;
        int l = min(DataLen, NRF_ESB_MAX_PAYLOAD_LENGTH);
        memset(payload, 0, sizeof(nrf_esb_payload_t));
        memcpy(payload->data, pData, l);
        payload->length = l;
        payload->noack = false;

        DataLen -= l;
        pData += l;
        cnt += l;
    }

    if (s_bEsbTxReady == true)
    {
        payload = (nrf_esb_payload_t *)CFifoGet(intrf->hTxFifo);
        if (payload != NULL)
        {
            s_bEsbTxReady = false;
            uint32_t res = nrf_esb_write_payload(payload);
            if (res != NRF_ERROR_NO_MEM)
            {
                s_bEsbTxReady = true;
            }
        }
    }

    return cnt;
}

/**
 * @brief - StopTx
 *      Completion of sending data via TxData.  Do require post processing
 * after all data was transmitted via TxData.
 * This function must clear the busy state for re-entrancy
 *
 * @param
 *      pDevIntrf : Pointer to an instance of the Device Interface
 *
 * @return  None
 */
void EsbIntrfStopTx(DEVINTRF *pDevIntrf)
{

}

/**
 * @brief - Reset
 *      This function perform a reset of interface.  Must provide empty
 * function of not used.
 *
 * @param
 *      pDevIntrf : Pointer to an instance of the Device Interface
 *
 * @return  None
 */
void EsbIntrfReset(DEVINTRF *pDevIntrf)
{

}

void nRFEsbEventHandler(nrf_esb_evt_t const * p_event)
{
    if (s_pEsbDevice == NULL)
        return;

    nrf_esb_payload_t *payload = NULL;
    DEVINTRF *devintrf = (DEVINTRF*)*s_pEsbDevice;
    ESBINTRF *esbintrf = (ESBINTRF *)devintrf->pDevData;

    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_FAILED:
            nrf_esb_flush_tx();
            nrf_esb_start_tx();
            break;
        case NRF_ESB_EVENT_TX_SUCCESS:
            payload = (nrf_esb_payload_t*)CFifoGet(esbintrf->hTxFifo);
            if (payload)
            {
                nrf_esb_write_payload(payload);
            }
            else
            {
                s_bEsbTxReady = true;
            }
            if (esbintrf->DevIntrf.EvtCB)
            {
                esbintrf->DevIntrf.EvtCB(&esbintrf->DevIntrf, DEVINTRF_EVT_TX_READY, NULL, 0);
            }
            break;

        case NRF_ESB_EVENT_RX_RECEIVED:
            payload = (nrf_esb_payload_t*)CFifoPut(esbintrf->hRxFifo);
            if (payload)
            {
                while(nrf_esb_read_rx_payload(payload) == NRF_SUCCESS);
            }
            if (esbintrf->DevIntrf.EvtCB)
            {
                esbintrf->DevIntrf.EvtCB(&esbintrf->DevIntrf, DEVINTRF_EVT_RX_DATA, NULL, 0);
            }
            break;
    }
}

bool EsbIntrfInit(ESBINTRF *pEsbIntrf, const ESBINTRF_CFG *pCfg)
{
    if (pEsbIntrf == NULL || pCfg == NULL)
        return false;

    uint32_t err_code;
    bool retval = false;
    nrf_esb_config_t esbcfg = NRF_ESB_DEFAULT_CONFIG;

    if (pCfg->PacketSize <= 0 || pCfg->pRxFifoMem == NULL || pCfg->pTxFifoMem == NULL)
    {
        esbcfg.payload_length = NRF_ESB_MAX_PAYLOAD_LENGTH;
        pEsbIntrf->hRxFifo = CFifoInit(s_EsbRxFifoMem, sizeof(s_EsbRxFifoMem), sizeof(nrf_esb_payload_t), true);
        pEsbIntrf->hTxFifo = CFifoInit(s_EsbTxFifoMem, sizeof(s_EsbTxFifoMem), sizeof(nrf_esb_payload_t), true);
    }
    else
    {
        esbcfg.payload_length = pCfg->PacketSize;
        pEsbIntrf->hRxFifo = CFifoInit(pCfg->pRxFifoMem, pCfg->RxFifoMemSize, sizeof(nrf_esb_payload_t), true);
        pEsbIntrf->hTxFifo = CFifoInit(pCfg->pTxFifoMem, pCfg->TxFifoMemSize, sizeof(nrf_esb_payload_t), true);
    }

    if (pCfg->Rate < 1000000)
    {
        pEsbIntrf->Rate = NRF_ESB_BITRATE_250KBPS;
    }
    else if (pCfg->Rate < 2000000)
    {
        pEsbIntrf->Rate = NRF_ESB_BITRATE_1MBPS;
    }
    else
    {
        pEsbIntrf->Rate = NRF_ESB_BITRATE_2MBPS;
    }

    esbcfg.bitrate = pEsbIntrf->Rate;
    esbcfg.protocol = pCfg->Protocol;
    esbcfg.mode = pCfg->Mode;
    esbcfg.tx_output_power = pCfg->TxPower;
    esbcfg.payload_length = pCfg->PacketSize;
    esbcfg.retransmit_count = pCfg->NbRetry;
    esbcfg.event_handler = nRFEsbEventHandler;
    esbcfg.selective_auto_ack = true;

    err_code = nrf_esb_init(&esbcfg);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(pCfg->BaseAddr0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(pCfg->BaseAddr1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(pCfg->PipePrefix, 8);
    VERIFY_SUCCESS(err_code);

    pEsbIntrf->DevIntrf.pDevData = (void*)pEsbIntrf;
    pEsbIntrf->DevIntrf.Enable = EsbIntrfEnable;
    pEsbIntrf->DevIntrf.Disable = EsbIntrfDisable;
    pEsbIntrf->DevIntrf.GetRate = EsbIntrfGetRate;
    pEsbIntrf->DevIntrf.SetRate = EsbIntrfSetRate;
    pEsbIntrf->DevIntrf.StartRx = EsbIntrfStartRx;
    pEsbIntrf->DevIntrf.RxData = EsbIntrfRxData;
    pEsbIntrf->DevIntrf.StopRx = EsbIntrfStopRx;
    pEsbIntrf->DevIntrf.StartTx = EsbIntrfStartTx;
    pEsbIntrf->DevIntrf.TxData = EsbIntrfTxData;
    pEsbIntrf->DevIntrf.StopTx = EsbIntrfStopTx;
    pEsbIntrf->DevIntrf.bBusy = false;
    pEsbIntrf->DevIntrf.MaxRetry = 0;
    pEsbIntrf->DevIntrf.EvtCB = pCfg->EvtCB;

    memcpy(&pEsbIntrf->EsbCfg, &esbcfg, sizeof(nrf_esb_config_t));

    return true;
}

bool EsbIntrf::Init(const ESBINTRF_CFG &Cfg)
{
    bool retval = EsbIntrfInit(&vEsbIntrf, &Cfg);

    if (retval)
    {
        s_pEsbDevice = this;
    }

    return retval;
}

EsbIntrf::EsbIntrf()
{

}

EsbIntrf::~EsbIntrf()
{
    nrf_esb_disable();

    s_pEsbDevice = NULL;
}
