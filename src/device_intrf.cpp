/*--------------------------------------------------------------------------
File   : device_intrf.cpp

Author : Hoang Nguyen Hoan          				Dec. 25, 2011

Desc   : Generic device interface class
		 This class is used to implement serial communication interfaces
		 such as I2C, UART, etc...  Not limited to wired interface

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
Modified by          Date              	Description
Hoan				Mar. 3, 2017		Rename to DevIntrf
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
int DeviceIntrfRx(DEVINTRF *pDev, int DevAddr, uint8_t *pBuff, int BuffLen)
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

int DeviceIntrfTx(DEVINTRF *pDev, int DevAddr, uint8_t *pBuff, int BuffLen)
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

int DeviceIntrfRead(DEVINTRF *pDev, int DevAddr, uint8_t *pAdCmd, int AdCmdLen,
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
                count = pDev->TxData(pDev, pAdCmd, AdCmdLen);
            }
            // Note : this is restart condition in read mode,
            // must not generate any stop condition here
            pDev->StartRx(pDev, DevAddr);

           	count = pDev->RxData(pDev, pRxBuff, RxLen);

           	DeviceIntrfStopRx(pDev);
        }
    } while (count <= 0 && nrtry-- > 0);

    return count;
}

int DeviceIntrfWrite(DEVINTRF *pDev, int DevAddr, uint8_t *pAdCmd, int AdCmdLen,
                  uint8_t *pTxData, int TxLen)
{
    int count = 0;
    int nrtry = pDev->MaxRetry;
    uint8_t d[AdCmdLen + TxLen];

    if (pTxData == NULL || pAdCmd == NULL)
        return 0;

    memcpy(d, pAdCmd, AdCmdLen);
    memcpy(&d[AdCmdLen], pTxData, TxLen);

    do {
        if (DeviceIntrfStartTx(pDev, DevAddr))
        {
            count = pDev->TxData(pDev, d, AdCmdLen + TxLen);
            DeviceIntrfStopTx(pDev);
        }
    } while (count <= 0 && nrtry-- > 0);

    if (count >= AdCmdLen)
        count -= AdCmdLen;
    else
        count = 0;

    return count;
}

