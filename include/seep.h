/*--------------------------------------------------------------------------
File   : seep.h

Author : Hoang Nguyen Hoan          Sept. 15, 2011

Desc   : Serial EEPROM device implementation

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
#ifndef __SEEP_H__
#define __SEEP_H__

#include <stdbool.h>

#include "serialintrf.h"
#include "iopincfg.h"

#pragma pack(push,4)

/**
 * @brief SEEP callback function
 * 		  This is a general callback function hook for special purpose
 * 		  defined in SEEP_CFG
 *
 * @param DevAddr : Device address
 * 		  pInterf : Pointer to physical interface connected to the device
 */
typedef bool (*SEEPCB)(int DevAddr, SERINTRFDEV *pInterf);

typedef struct _Seep_Config {
	int DevAddr;	// Device address
	int AddrLen;	// Serial EEPROM memory address length in bytes
	int PageSize;	// Wrap around page size in bytes
	SEEPCB pInitCB;	// For custom initialization. Set to NULL if not used
	SEEPCB pWaitCB;	// If provided, this is called when there are long delays
					// for a device to complete its write cycle
   					// This is to allow application to perform other tasks
   					// while waiting. Set to NULL is not used
	IOPINCFG WrProtPin; // if Write protect pin is not used, set {-1, -1, }
	                    // This pin is assumed active high,
	                    // ie. Set to 1 to enable Write Protect
} SEEP_CFG;

typedef struct {
	int DevAddr;	// Device address
	int AddrLen;	// Serial EEPROM memory address length in bytes
	int PageSize;	// Wrap around page size
	IOPINCFG WrProtPin; // Write protect I/O pin
	SERINTRFDEV	*pInterf;
	SEEPCB pWaitCB;	// If provided, this is called when there are long delays
					// for a device to complete its write cycle
   					// This is to allow application to perform other tasks
   					// while waiting. Set to NULL is not used
} SEEPDEV;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize Serial EEPROM driver
 *
 * @param   pDev     : Pointer to driver data to be initialized
 *          pCfgData : Pointer to serial EEPROM configuration data
 *          pInterf  : Pointer to the interface device on which the SEEPROM
 *                     is connected to
 *
 * @return  true - initialization successful
 */
bool SeepInit(SEEPDEV *pDev, SEEP_CFG *pCfgData, SERINTRFDEV *pInterf);

/**
 * @brief Read Serial EEPROM data
 *
 * @param   pDev     : Pointer to driver data
 *          Address  : Memory address to read
 *          pBuff    : Pointer to buffer to receive data
 *          Len      : Size of the buffer in bytes
 *
 * @return  Number of bytes read
 */
int SeepRead(SEEPDEV *pDev, int Addr, uint8_t *pBuff, int Len);

/**
 * @brief Write to data to Serial EEPROM
 *
 * @param   pDev     : Pointer to driver data
 *          Address  : Memory address to write
 *          pData    : Pointer to data to write
 *          Len      : Number of bytes to write
 *
 * @return  Number of bytes written
 */
int SeepWrite(SEEPDEV *pDev, int Addr, uint8_t *pData, int Len);

/**
 * @brief Set the write protect pin
 *
 * @param   pDev     : Pointer to driver data
 *          bVal     : true - Enable write protect
 *                     false - Disable write protect
 */
void SeepSetWriteProt(SEEPDEV *pDev, bool bVal);

#ifdef __cplusplus
}

class Seep {
public:
    Seep();
    virtual ~Seep();
    Seep(Seep&);    // copy ctor not allowed

    virtual bool Init(SEEP_CFG &Cfg, SerialIntrf *pInterf);
    virtual void Set(int DevAddr, int PageSize, int AddrLen) {
        vDevData.DevAddr = DevAddr;
        vDevData.PageSize = PageSize;
        vDevData.AddrLen = AddrLen;
    }
    virtual int Read(int Addr, uint8_t *pBuff, int Len) { return SeepRead(&vDevData, Addr, pBuff, Len); }
    virtual int Write(int Addr, uint8_t *pData, int Len) { return SeepWrite(&vDevData, Addr, pData, Len); }

    operator SEEPDEV* () { return &vDevData; }

protected:
    SEEPDEV vDevData;
};

#endif

#endif	// __SEEP_H__

