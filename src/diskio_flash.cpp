/*--------------------------------------------------------------------------
File   : diskio_flash.cpp

Author : Hoang Nguyen Hoan          Aug. 30, 2016

Desc   : Generic flash disk I/O driver class

Copyright (c) 2016, Motsai, all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : nh.hoang at motsai dot com

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
#include "diskio_flash.h"

bool FlashDiskIO::Init(FLASHDISKIO_CFG &Cfg, SerialIntrf *pInterf,
                       DISKIO_CACHE_DESC *pCacheBlk, int NbCacheBlk)
{
    if (pInterf == NULL)
        return false;

    if (Cfg.FlashInit)
    {
        if (Cfg.FlashInit(Cfg.DevNo, pInterf) == false)
            return false;
    }

    vDevNo          = Cfg.DevNo;
    vEraseSize      = Cfg.EraseSize;
    if (Cfg.WriteSize == 0)
        vWriteSize = DISKIO_SECT_SIZE;
    else
        vWriteSize      = Cfg.WriteSize;
    vTotalSize      = Cfg.TotalSize;
    vAddrSize       = Cfg.AddrSize;
    vpInterf        = pInterf;


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

bool FlashDiskIO::WaitReady(uint32_t Timeout)
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
    int cnt = vpInterf->Tx(vDevNo, &d, 1);

    return false;
}

void FlashDiskIO::Erase()
{
    uint8_t d;

    WriteEnable();
    WaitReady();

    d = FLASH_CMD_BULK_ERASE;

    int cnt = vpInterf->Tx(vDevNo, &d, 1);

    WaitReady(-1);
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
    uint8_t *p = (uint8_t*)BlkNo;

    WriteEnable();
    WaitReady();

    d[0] = FLASH_CMD_BLOCK_ERASE;

    for (int i = 0; i < NbBlk; i++)
    {
        for (int i = 1; i <= vAddrSize; i++)
            d[i] = p[vAddrSize - i];
        vpInterf->Tx(vDevNo, d, 4);
        BlkNo += vEraseSize;
    }
    WaitReady(-1);
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

    WaitReady(100000);

    d[0] = FLASH_CMD_READ;

    while (cnt > 0)
    {
        for (int i = 1; i <= vAddrSize; i++)
            d[i] = p[vAddrSize - i];

        vpInterf->StartRx(vDevNo);
        vpInterf->TxData((uint8_t*)d, 4);
        int l = vpInterf->RxData(pBuff, DISKIO_SECT_SIZE);
        vpInterf->StopRx();
        if (l <= 0)
            return false;
        cnt -= l;
        addr += l;
        pBuff += l;
    }
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

    WaitReady();
    WriteEnable();

    d[0] = FLASH_CMD_WRITE;

    cnt = DISKIO_SECT_SIZE;
    while (cnt > 0)
    {
        for (int i = 1; i <= vAddrSize; i++)
            d[i] = p[vAddrSize - i];

        int l = min(cnt, vWriteSize);
        vpInterf->StartTx(vDevNo);
        vpInterf->TxData((uint8_t*)d, 4);
        l = vpInterf->TxData(pData, l);
        vpInterf->StopTx();
        if (l <= 0)
            false;
        cnt -= l;
        pData += l;
        addr += l;
    }
    WriteDisable();

    return true;
}


