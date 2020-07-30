/**-------------------------------------------------------------------------
@file	device_intrf.h

@brief	Generic data transfer interface class

This class is used to implement device communication interfaces such as I2C, UART, etc...
Not limited to wired or physical interface.  It could be soft interface as well such
as SLIP protocol or any mean of transferring data between 2 entities.

@author	Hoang Nguyen Hoan
@date	Nov. 25, 2011

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
#include <string.h>

#include "device_intrf.h"

// NOTE : For thread safe use
//
// DeviceIntrfStartRx
// DeviceIntrfStopRx
// DeviceIntrfStartTx
// DeviceIntrfStopTx
//
int DeviceIntrfRx(DEVINTRF * const pDev, uint32_t DevAddr, uint8_t *pBuff, int BuffLen)
{
	if (pBuff == NULL || BuffLen <= 0)
		return 0;

	int count = 0;
	int nrtry = pDev->MaxRetry;

	do {
		if (DeviceIntrfStartRx(pDev, DevAddr)) {
			count = pDev->RxData(pDev, pBuff, BuffLen);
			DeviceIntrfStopRx(pDev);
		}
	} while(count <= 0 && nrtry-- > 0);

	return count;
}

int DeviceIntrfTx(DEVINTRF * const pDev, uint32_t DevAddr, uint8_t *pBuff, int BuffLen)
{
	if (pBuff == NULL || BuffLen <= 0)
		return 0;

	int count = 0;
	int nrtry = pDev->MaxRetry;

	do {
		if (DeviceIntrfStartTx(pDev, DevAddr)) {
			count = pDev->TxData(pDev, pBuff, BuffLen);
			DeviceIntrfStopTx(pDev);
		}
	} while (count <= 0 && nrtry-- > 0);

	return count;
}

int DeviceIntrfRead(DEVINTRF * const pDev, uint32_t DevAddr, uint8_t *pAdCmd, int AdCmdLen,
                 uint8_t *pRxBuff, int RxLen)
{
    int count = 0;
    int nrtry = pDev->MaxRetry;

    if (pRxBuff == NULL || RxLen <= 0)
        return 0;

    do {
        if (DeviceIntrfStartTx(pDev, DevAddr))
        {
            if (pAdCmd)
            {
            	if (pDev->TxSrData)
            	{
            		count = pDev->TxSrData(pDev, pAdCmd, AdCmdLen);
            	}
            	else
            	{
            		count = pDev->TxData(pDev, pAdCmd, AdCmdLen);
            	}
            }
           // if (pDev->TxSrData)
            {
            	// Note : this is restart condition in read mode,
            	// must not generate any stop condition here
            	pDev->StartRx(pDev, DevAddr);
            }
           	count = pDev->RxData(pDev, pRxBuff, RxLen);

           	DeviceIntrfStopRx(pDev);
        }
    } while (count <= 0 && nrtry-- > 0);

    return count;
}

int DeviceIntrfWrite(DEVINTRF * const pDev, uint32_t DevAddr, uint8_t *pAdCmd, int AdCmdLen,
                  uint8_t *pData, int DataLen)
{
    int count = 0, txlen = AdCmdLen;
    int nrtry = pDev->MaxRetry;

    if (pAdCmd == NULL || (AdCmdLen + DataLen) <= 0)
        return 0;

#if defined(WIN32) || defined(__ICCARM__)
	uint8_t d[100];
#else
	uint8_t d[AdCmdLen + DataLen];
#endif

	// NOTE : Some I2C devices that uses DMA transfer may require that the tx to be combined
    // into single tx. Because it may generate a end condition at the end of the DMA
    memcpy(d, pAdCmd, AdCmdLen);

    if (pData != NULL && DataLen > 0)
    {
    	memcpy(&d[AdCmdLen], pData, DataLen);
    	txlen += DataLen;
    }

    do {
        if (DeviceIntrfStartTx(pDev, DevAddr))
        {
            count = pDev->TxData(pDev, d, txlen);
			DeviceIntrfStopTx(pDev);
        }
    } while (count <= 0 && nrtry-- > 0);

    if (count >= AdCmdLen)
        count -= AdCmdLen;
    else
        count = 0;

    return count;
}

