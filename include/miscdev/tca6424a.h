/**-------------------------------------------------------------------------
@file	tca6424a.h

@brief	TCA6424A Low-Voltage 24-Bit I2C and SMBus I/O Expander

@author	Hoang Nguyen Hoan
@date	Oct. 7, 2018

@license

Copyright (c) 2018, I-SYST inc., all rights reserved

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
#ifndef __TCA6424A_H__
#define __TCA6424A_H__

#include <stdint.h>

typedef struct __Tca6424a_Cfg {
	uint8_t DevAddr,
} TCA6424A_CFG;

class IoExpTca6424a : public Device {
public:
	bool Init(TCA6424A_CFG &CfgData, DeviceIntrf * const pIntrf);
	/**
	 * @brief	Power on or wake up device
	 *
	 * @return	true - If success
	 */
	virtual bool Enable();

	/**
	 * @brief	Put device in power down or power saving sleep mode
	 *
	 * @return	None
	 */
	virtual void Disable();

	/**
	 * @brief	Reset device to it initial state
	 *
	 * @return	None
	 */
	virtual void Reset();

protected:
private:
};



#endif // __TCA6424A_H__
