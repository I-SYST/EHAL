/*--------------------------------------------------------------------------
File   : device.cpp

Author : Hoang Nguyen Hoan          			Feb. 12, 2017

Desc   : Generic device base class

Copyright (c) 2017, I-SYST inc., all rights reserved

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

#include "device.h"

uint8_t Device::ReadReg8(uint8_t *pRegAddr, int RegAddrLen)
{
	uint8_t val = 0;

	Read(pRegAddr, RegAddrLen, &val, 1);

	return val;
}

uint16_t Device::ReadReg16(uint8_t *pRegAddr, int RegAddrLen)
{
	uint16_t val = 0;

	Read(pRegAddr, RegAddrLen,(uint8_t*) &val, 2);

	return val;
}

uint32_t Device::ReadReg32(uint8_t *pRegAddr, int RegAddrLen)
{
	uint32_t val = 0;

	Read(pRegAddr, RegAddrLen, (uint8_t*)&val, 4);

	return val;
}

bool Device::WriteReg(uint8_t *pRegAddr, int RegAddrLen, uint8_t Data)
{
	return Write(pRegAddr, RegAddrLen, &Data, 1) > 0;
}

bool Device::WriteReg(uint8_t *pRegAddr, int RegAddrLen, uint16_t Data)
{
	return Write(pRegAddr, RegAddrLen, (uint8_t*)&Data, 2) > 1;
}

bool Device::WriteReg(uint8_t *pRegAddr, int RegAddrLen, uint32_t Data)
{
	return Write(pRegAddr, RegAddrLen, (uint8_t*)&Data, 1) > 3;
}

