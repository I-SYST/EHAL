/**-------------------------------------------------------------------------
@file	diskio_flash.cpp

@brief	Generic flash disk I/O driver class


@author	Hoang Nguyen Hoan
@date	Aug. 30, 2016


@license

MIT License

Copyright (c) 2016 I-SYST inc. All rights reserved.

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
#include <stdio.h>

#include "istddef.h"
#include "diskio_flash.h"
#include "idelay.h"

FlashDiskIO::FlashDiskIO() : DiskIO()
{
	vpWaitCB = NULL;
	vpInterf = NULL;
}

bool FlashDiskIO::Init(FLASHDISKIO_CFG &Cfg, DeviceIntrf * const pInterf,
                       DISKIO_CACHE_DESC * const pCacheBlk, int NbCacheBlk)
{
    if (pInterf == NULL)
        return false;

    if (Cfg.pInitCB)
    {
        if (Cfg.pInitCB(Cfg.DevNo, pInterf) == false)
            return false;
    }

    if (Cfg.pWaitCB)
    	vpWaitCB = Cfg.pWaitCB;

    vDevNo          = Cfg.DevNo;
    vEraseSize      = Cfg.EraseSize;
    if (Cfg.WriteSize == 0)
        vWriteSize = DISKIO_SECT_SIZE;
    else
        vWriteSize = Cfg.WriteSize;
    vTotalSize      = Cfg.TotalSize;
    vAddrSize       = Cfg.AddrSize;
    vpInterf        = pInterf;

    uint32_t d = ReadId();

    if (pCacheBlk && NbCacheBlk > 0)
    {
        SetCache(pCacheBlk, NbCacheBlk);
    }

    return true;
}

uint32_t FlashDiskIO::ReadId()
{
	uint32_t id = -1;
	uint8_t cmd;

	WaitReady();

	cmd = FLASH_CMD_READID;

	vpInterf->StartRx(vDevNo);
    vpInterf->TxData(&cmd, 1);
    vpInterf->RxData((uint8_t*)&id, 4);
    vpInterf->StopRx();

    return id;
}

uint8_t FlashDiskIO::ReadStatus()
{
    uint8_t d;

    d = FLASH_CMD_READSTATUS;
    vpInterf->StartRx(vDevNo);
    vpInterf->TxData(&d, 1);
    vpInterf->RxData(&d, 1);
    vpInterf->StopRx();

    return d;
}

bool FlashDiskIO::WaitReady(uint32_t Timeout, uint32_t usRtyDelay)
{
    uint8_t d;

    do {
        d = FLASH_CMD_READSTATUS;
        vpInterf->StartRx(vDevNo);
        vpInterf->TxData(&d, 1);
        vpInterf->RxData(&d, 1);
        vpInterf->StopRx();
        if (!(d & FLASH_STATUS_WIP))
            return true;

        if (usRtyDelay > 0)
        {
            if (vpWaitCB)
            	vpWaitCB(vDevNo, vpInterf);
            else
            	usDelay(usRtyDelay);
        }

    } while (Timeout-- > 0);

    return false;
}

void FlashDiskIO::WriteDisable()
{
    uint8_t d = FLASH_CMD_WRDISABLE;
    vpInterf->Tx(vDevNo, &d, 1);
}

bool FlashDiskIO::WriteEnable(uint32_t Timeout)
{
    uint8_t d;

    WaitReady(Timeout);

    d = FLASH_CMD_WRENABLE;
    vpInterf->Tx(vDevNo, &d, 1);

    return false;
}

void FlashDiskIO::Erase()
{
    uint8_t d;

    WriteEnable();
    WaitReady();

    d = FLASH_CMD_BULK_ERASE;

    vpInterf->Tx(vDevNo, &d, 1);

    // This is a long wait polling at every second only
    WaitReady(-1, 1000000);
    WriteDisable();
}

/**
 * Erase Flash block.
 *
 * @param   BlkNo   : Starting block number to erase.
 *          NbBlk   : Number of consecutive blocks to erase
 */
void FlashDiskIO::EraseBlock(uint32_t BlkNo, int NbBlk)
{
    uint8_t d[8];

    BlkNo *= vEraseSize;
    uint8_t *p = (uint8_t*)&BlkNo;

    d[0] = FLASH_CMD_BLOCK_ERASE;

    for (int k = 0; k < NbBlk; k++)
    {
        for (int i = 1; i <= vAddrSize; i++)
            d[i] = p[vAddrSize - i];
        WaitReady(-1, 100);

        // Need to re-enable write here, because some flash
        // devices may reset write enable after a write
        // complete
        WriteEnable();

        vpInterf->Tx(vDevNo, d, vAddrSize + 1);
        BlkNo += vEraseSize;
    }
    WriteDisable();
}

/**
 * Read one sector from physical device
 */
bool FlashDiskIO::SectRead(uint32_t SectNo, uint8_t *pBuff)
{
    uint8_t d[9];
    uint32_t addr = SectNo * DISKIO_SECT_SIZE;
    uint8_t *p = (uint8_t*)&addr;
    int cnt = DISKIO_SECT_SIZE;

    // Makesure there is no write access pending
    WaitReady(100000);

    d[0] = FLASH_CMD_READ;

    while (cnt > 0)
    {
        for (int i = 1; i <= vAddrSize; i++)
            d[i] = p[vAddrSize - i];

        vpInterf->StartRx(vDevNo);
        vpInterf->TxData((uint8_t*)d, vAddrSize + 1);
        int l = vpInterf->RxData(pBuff, DISKIO_SECT_SIZE);
        vpInterf->StopRx();
        if (l <= 0)
            return false;
        cnt -= l;
        addr += l;
        pBuff += l;
    }
    //printf("RSect : %d 0x%02x 0x%02x 0x%02x 0x%02x\r\n", SectNo, pBuff[0], pBuff[1], pBuff[2], pBuff[3]);
    return true;
}

/**
 * Write one sector to physical device
 */
bool FlashDiskIO::SectWrite(uint32_t SectNo, uint8_t *pData)
{
    uint8_t d[9];
    uint32_t addr = SectNo * DISKIO_SECT_SIZE;
    uint8_t *p = (uint8_t*)&addr;

    int cnt = 0;

   // printf("Sect : %d 0x%02x 0x%02x 0x%02x 0x%02x\r\n", SectNo, pData[0], pData[1], pData[2], pData[3]);
    d[0] = FLASH_CMD_WRITE;

    cnt = DISKIO_SECT_SIZE;
    while (cnt > 0)
    {
        for (int i = 1; i <= vAddrSize; i++)
            d[i] = p[vAddrSize - i];

        int l = min(cnt, vWriteSize);

        WaitReady();

        // Some Flash will reset write enable bit at completion
        // when page size is less than 512 bytes.
        // We need to set it again
        WriteEnable();

        vpInterf->StartTx(vDevNo);
        vpInterf->TxData((uint8_t*)d, vAddrSize + 1);
        l = vpInterf->TxData(pData, l);
        vpInterf->StopTx();
        if (l <= 0)
            return false;
        cnt -= l;
        pData += l;
        addr += l;
    }
    WriteDisable();

    return true;
}


