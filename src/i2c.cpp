/*--------------------------------------------------------------------------
File   : i2c.cpp

Author : Hoang Nguyen Hoan          June 9, 2011

Desc   : i2c driver

Copyright (c) 2011, I-SYST, all rights reserved

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
#include <string.h>

#include "istddef.h"
#include "idelay.h"
#include "iopincfg.h"
#include "iopinctrl.h"
#include "i2c.h"

void I2CBusReset(I2CDEV * const pDev)
{
	if (pDev == NULL)
	{
		return;
	}

	int sclport = pDev->Pins[I2C_SCL_IOPIN_IDX].PortNo;
	int sclpin = pDev->Pins[I2C_SCL_IOPIN_IDX].PinNo;
	int sdaport = pDev->Pins[I2C_SDA_IOPIN_IDX].PortNo;
	int sdapin = pDev->Pins[I2C_SDA_IOPIN_IDX].PinNo;

	pDev->DevIntrf.Disable(&pDev->DevIntrf);

    IOPinConfig(sclport, sclpin, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
    IOPinConfig(sdaport, sdapin, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);

    IOPinSet(sdaport, sdapin);

    for (int i = 0; i < 10; i++)
    {
        IOPinSet(sclport, sclpin);
        usDelay(5);
        IOPinClear(sclport, sclpin);
        usDelay(5);
    }

    IOPinConfig(sdaport, sdapin, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
    IOPinClear(sdaport, sdapin);
    usDelay(5);
    IOPinSet(sclport, sclpin);
    usDelay(2);
    IOPinSet(sdaport, sdapin);

    pDev->DevIntrf.Enable(&pDev->DevIntrf);
}
