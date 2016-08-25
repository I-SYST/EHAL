/*---------------------------------------------------------------------------
File : lpc11uxx_iap.c

Author : Hoang Nguyen Hoan          Jan. 26, 2015

Desc : LPC11Uxx IAP functions

Copyright (c) 2015, I-SYST inc, all rights reserved

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

#include "iap_lpc11uxx.h"

#define IAP_LOCATION 0x1fff1ff1

typedef void (*IAPENTRY)(IAP_CMD_STATUS *pCmd, IAP_CMD_STATUS *pRes);
typedef void (*IAPENTRY1)(uint32_t *pCmd, IAP_CMD_STATUS *pRes);

extern uint32_t SystemCoreClock;

IAPENTRY IapEntry = (IAPENTRY)IAP_LOCATION;
IAPENTRY1 IapEntry1 = (IAPENTRY1)IAP_LOCATION;

IAPSTATUS IAPPrepSectWrite(int StartSect, int EndSect)
{
	IAP_CMD_STATUS cmd;
	IAP_CMD_STATUS res;


	cmd.CmdStatus = IAPCMD_PREP_SECTOR;
	cmd.Data[0] = StartSect;
	cmd.Data[1] = EndSect;

	IapEntry(&cmd, &res);

	return res.CmdStatus;
}

IAPSTATUS IAPCopyRamToFlash(uint8_t *pDst, uint8_t *pSrc, int Size)
{
	IAP_CMD_STATUS cmd;
	IAP_CMD_STATUS res;

	cmd.CmdStatus = IAPCMD_CPY_RAM_TO_FLASH;
	cmd.Data[0] = (uint32_t)pDst;
	cmd.Data[1] = (uint32_t)pSrc;
	cmd.Data[2] = Size;
	cmd.Data[3] = SystemCoreClock / 1000;

	IapEntry(&cmd, &res);

	return res.CmdStatus;
}

IAPSTATUS IAPEraseSector(int StartSect, int EndSect)
{
	IAP_CMD_STATUS cmd;
	IAP_CMD_STATUS res;

	cmd.CmdStatus = IAPCMD_ERASE_SECT;
	cmd.Data[0] = StartSect;
	cmd.Data[1] = EndSect;
	cmd.Data[2] = SystemCoreClock / 1000;

	IapEntry(&cmd, &res);

	return res.CmdStatus;
}

IAPSTATUS IAPBlankCheck(int StartSect, int EndSect, IAP_CMD_STATUS *pRes)
{
	IAP_CMD_STATUS cmd;

	cmd.CmdStatus = IAPCMD_BLANK_CHECK;
	cmd.Data[0] = StartSect;
	cmd.Data[1] = EndSect;

	IapEntry(&cmd, pRes);

	return pRes->CmdStatus;
}

IAPSTATUS IAPReadPartId(IAP_CMD_STATUS *pRes)
{
	IAP_CMD_STATUS cmd;

	cmd.CmdStatus = IAPCMD_READ_PARTID;

	IapEntry(&cmd, pRes);

	return pRes->CmdStatus;
}

IAPSTATUS IAPReadBootCodeVers(IAP_CMD_STATUS *pRes)
{
	IAP_CMD_STATUS cmd;

	cmd.CmdStatus = IAPCMD_READ_BOOTCODE_VER;

	IapEntry(&cmd, pRes);

	return pRes->CmdStatus;
}

IAPSTATUS IAPCompare(uint8_t *pDst, uint8_t *pSrc, int Size, IAP_CMD_STATUS *pRes)
{
	IAP_CMD_STATUS cmd;

	cmd.CmdStatus = IAPCMD_COMPARE;
	cmd.Data[0] = (uint32_t)pDst;
	cmd.Data[1] = (uint32_t)pSrc;
	cmd.Data[2] = Size;

	IapEntry(&cmd, pRes);

	return pRes->CmdStatus;
}

IAPSTATUS IAPJumpISP()
{
	IAP_CMD_STATUS cmd;
	IAP_CMD_STATUS res;

	cmd.CmdStatus = IAPCMD_INVOKE_ISP;

	IapEntry(&cmd, &res);

	return res.CmdStatus;
}

IAPSTATUS IAPReadUID(IAP_CMD_STATUS *pRes)
{
	IAP_CMD_STATUS cmd;

	cmd.CmdStatus = IAPCMD_READ_UID;

	IapEntry(&cmd, pRes);

	return pRes->CmdStatus;
}

IAPSTATUS IAPErasePage(int StartPage, int EndPage)
{
	IAP_CMD_STATUS cmd;
	IAP_CMD_STATUS res;

	cmd.CmdStatus = IAPCMD_ERASE_PAGE;
	cmd.Data[0] = StartPage;
	cmd.Data[1] = EndPage;
	cmd.Data[2] = SystemCoreClock / 1000;

	IapEntry(&cmd, &res);

	return res.CmdStatus;
}

IAPSTATUS IAPEepromWrite(uint32_t EepAddr, uint8_t *pData, int Size)
{
	IAP_CMD_STATUS cmd;
	IAP_CMD_STATUS res;

	cmd.CmdStatus = IAPCMD_EEPROM_WRITE;
	cmd.Data[0] = EepAddr;
	cmd.Data[1] = (uint32_t)pData;
	cmd.Data[2] = Size;
	cmd.Data[3] = SystemCoreClock / 1000;

	IapEntry(&cmd, &res);

	return res.CmdStatus;
}

IAPSTATUS IAPEepromRead(uint32_t EepAddr, uint8_t *pData, int Size)
{
	IAP_CMD_STATUS cmd;
	IAP_CMD_STATUS res;

	cmd.CmdStatus = IAPCMD_EEPROM_READ;
	cmd.Data[0] = EepAddr;
	cmd.Data[1] = (uint32_t)pData;
	cmd.Data[2] = Size;
	cmd.Data[3] = SystemCoreClock / 1000;

	IapEntry(&cmd, &res);

	return res.CmdStatus;
}

