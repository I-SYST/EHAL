/**-------------------------------------------------------------------------
@file	dac_device.h

@brief	Generic DAC device

@author	Hoang Nguyen Hoan
@date	June 16, 2017

@license

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

----------------------------------------------------------------------------*/
#ifndef __DAC_DEVICE_H__
#define __DAC_DEVICE_H__

#include <stdint.h>

#include "device.h"

typedef enum __DAC_Mode {
	DAC_MODE_SINGLE_ENDED,
	DAC_MODE_DIFFERENTIAL
} DAC_MODE;

typedef struct __DAC_Config {
	DAC_MODE Mode;			// ADC mode
} DAC_CFG;

class DacDevice : virtual public Device {
public:
	virtual bool Init(const DAC_CFG &CfgData, DeviceIntrf * const pIntrf) = 0;

private:

};

#endif // __DAC_DEVICE_H__
