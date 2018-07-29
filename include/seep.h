/**-------------------------------------------------------------------------
@file	seep.h

@brief	Generic implementation of Serial EEPROM device

This implementation supports most Serial EEPROM.
	- Automatic memory block selections
	- Multi-bytes memory address length

There is no need to write code for each type of EEPROM.  Just fill EEPROM
information in the SEEP_CFG data structure then pass it to the init function.

Example of defining EEPROM info :

-----
CAT24C32 : 32Kbits, 2 byte address length, 32 bytes per page, Write delays 5 ms

static const SEEP_CFG s_CAT24C02EepCfg = {
	0x50,			// Device address
	2,				// Address length
	32,				// Page size
	32 * 1024 / 8,	// Total size in bytes
	5,				// Twr : 5 ms
};

-----
CAT24C02 : 2Kbits, 1 byte address length, 16 bytes per page, Write delays 5 ms

static const SEEP_CFG s_CAT24C02EepCfg = {
	0x50,		// Device address
	1,			// Address length
	16,			// Page size
	2048 / 8,	// Total size in bytes
	5,			// Twr : 5 ms
};

-----
M24C64S : 64Kbits, 2 bytes address length, 32 bytes per page, Write delays 5 ms

static const SEEP_CFG s_M24C64SEepCfg = {
	0x50,		// Device address
	2,			// Address length
	32,			// Page size
	64 * 1024 / 8,	// Total size in bytes
	5,			// Twr : 5 ms
};

-----
AT24CS08 : 8Kbits, 1 byte address length, 16 bytes per page, write delays 5 ms

static const SEEP_CFG s_AT24CS08EepCfg = {
	0x50,		// Device address
	1,			// Address length
	16,			// Page size
	1024,		// Total size in bytes
	5,			// Twr : 5 ms
};

-----
24AA08/24LC08B : 8Kbits, 1 byte address length, 16 bytes per page, write delays 3 ms

static const SEEP_CFG s_AT24CS08EepCfg = {
	0x50,		// Device address
	1,			// Address length
	16,			// Page size
	1024,		// Total size in bytes
	3,			// Twr : 3 ms
};

----
Usage in C++ :

// I2C interface instance to be used. Assuming it is already initialized.
I2C g_I2C;

// Declare instance
Seep g_Seep;

// Initialize
g_Seep.Init(s_AT24CS08EepCfg, &g_I2C);

// Read/Write
uint8_t buff[40];

g_Seep.Write(0x100, buff, 40);	// Write 40 bytes at address 0x100
g_Seep.Read(0x10, buff, 40); // Read 40 bytes from address 0x10

-----
Usage in C :

// I2C interface instance to be used. Assuming it is already initialized.
I2CDEV g_I2CDev;

// Declare instance
SEEPDEV g_SeepDev;

// Initialize
SeepInit(&g_SeepDev, &s_AT24CS08EepCfg, &g_I2CDev->DevIntrf);


// Read/Write
uint8_t buff[40];

SeepWrite(&g_SeepDev, 0x100, buff, 40);	// Write 40 bytes at address 0x100
SeepRead(&g_SeepDev, 0x10, buff, 40); // Read 40 bytes from address 0x10


@author	Hoang Nguyen Hoan
@date	Sep. 15, 2011

@license

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

----------------------------------------------------------------------------*/
#ifndef __SEEP_H__
#define __SEEP_H__

#include <stdbool.h>

#include "device_intrf.h"
#include "coredev/iopincfg.h"

/** @addtogroup Storage
  * @{
  */

#pragma pack(push,4)

/**
 * @brief SEEP callback function
 *
 * This is a general callback function hook for special purpose defined in SEEP_CFG
 *
 * @param	DevAddr : Device address.
 * @param	pInterf : Pointer to physical interface connected to the device
 *
 * @return	true - success
 */
typedef bool (*SEEPCB)(int DevAddr, DEVINTRF * const pInterf);

/// Structure defining Serial EEPROM device
typedef struct __Seep_Config {
    uint8_t DevAddr;	//<! Device address
    uint8_t AddrLen;	//<! Serial EEPROM memory address length in bytes
    uint16_t PageSize;	//<! Wrap around page size in bytes
    uint32_t Size;      //<! Total EEPROM size in bytes
    uint32_t WrDelay;   //<! Write delay time in msec
    IOPINCFG WrProtPin; //<! if Write protect pin is not used, set {-1, -1, }
                        //<! This pin is assumed active high,
                        //<! ie. Set to 1 to enable Write Protect
	SEEPCB pInitCB;	    //<! For custom initialization. Set to NULL if not used
	SEEPCB pWaitCB;	    //<! If provided, this is called when there are long delays
					    //<! for a device to complete its write cycle
   					    //<! This is to allow application to perform other tasks
   					    //<! while waiting. Set to NULL is not used
} SEEP_CFG;

/// @brief Device internal data.
///
/// Pointer to this structure serve as handle to the implementation function
typedef struct __Seep_Device {
	uint8_t DevAddr;    //<! Device address
	uint8_t AddrLen;    //<! Serial EEPROM memory address length in bytes
	uint16_t PageSize;	//<! Wrap around page size
	uint32_t Size;      //<! Total EEPROM size in bytes
	uint32_t WrDelay;   //<! Write delay in usec
	IOPINCFG WrProtPin; //<! Write protect I/O pin
	DEVINTRF *pInterf;  //<! Device interface
	SEEPCB pWaitCB;	    //<! If provided, this is called when there are long delays
					    //<! for a device to complete its write cycle
   					    //<! This is to allow application to perform other tasks
   					    //<! while waiting. Set to NULL is not used
} SEEPDEV;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize Serial EEPROM driver.
 *
 * @param	pDev     : Pointer to driver data to be initialized
 * @param	pCfgData : Pointer to serial EEPROM configuration data
 * @param	pInterf  : Pointer to the interface device on which the SEEPROM
 *                     is connected to
 *
 * @return  true - initialization successful
 */
bool SeepInit(SEEPDEV * const pDev, const SEEP_CFG *pCfgData, DEVINTRF * const pInterf);

/**
 * @brief Get EEPROM size.
 *
 * @param   pDev     : Pointer to driver data
 *
 * @return  Total size in bytes
 */
static inline uint32_t SeepGetSize(SEEPDEV * const pDev) {
    return pDev ? pDev->Size : 0;
}

/**
 * @brief Get EEPROM page size.
 *
 * @param   pDev     : Pointer to driver data
 *
 * @return  Page size in bytes
 */
static inline uint16_t SeepGetPageSize(SEEPDEV * const pDev) {
    return pDev? pDev->PageSize : 0;
}

/**
 * @brief Read Serial EEPROM data.
 *
 * @param   pDev     : Pointer to driver data
 * @param   Address  : Memory address to read
 * @param   pBuff    : Pointer to buffer to receive data
 * @param   Len      : Size of the buffer in bytes
 *
 * @return  Number of bytes read
 */
int SeepRead(SEEPDEV * const pDev, uint32_t Addr, uint8_t *pBuff, int Len);

/**
 * @brief Write to data to Serial EEPROM.
 *
 * @param   pDev     : Pointer to driver data
 * @param   Address  : Memory address to write
 * @param   pData    : Pointer to data to write
 * @param   Len      : Number of bytes to write
 *
 * @return  Number of bytes written
 */
int SeepWrite(SEEPDEV * const pDev, uint32_t Addr, uint8_t *pData, int Len);

/**
 * @brief Set the write protect pin.
 *
 * @param   pDev     : Pointer to driver data
 * @param   bVal     : true - Enable write protect
 *                     false - Disable write protect
 */
void SeepSetWriteProt(SEEPDEV * const pDev, bool bVal);

#ifdef __cplusplus
}

/// @brief	Generic Serial EEPROM implementation class
///
/// The thing to know about an EEPROM is its device address and its page size
/// Set those parameters in the SEEP_CFG data structure to initialize this class
class Seep {
public:
    Seep();
    virtual ~Seep();
    Seep(Seep&);    // copy ctor not allowed

    /**
     * @brief Initialize Serial EEPROM driver.
     *
     * @param	pCfgData : Pointer to serial EEPROM configuration data
     * @param	pInterf  : Pointer to the interface device on which the SEEPROM
     *                     is connected to
     *
     * @return  true - initialization successful
     */
    virtual bool Init(const SEEP_CFG &Cfg, DeviceIntrf * const pInterf) {
        return SeepInit(&vDevData, &Cfg, *pInterf);
    }

    /**
     * @brief Read Serial EEPROM data.
     *
     * @param   Address  : Memory address to read
     * @param   pBuff    : Pointer to buffer to receive data
     * @param   Len      : Size of the buffer in bytes
     *
     * @return  Number of bytes read
     */
    virtual int Read(uint32_t Addr, uint8_t *pBuff, int Len) { return SeepRead(&vDevData, Addr, pBuff, Len); }

    /**
     * @brief Write to data to Serial EEPROM.
     *
     * @param   Address  : Memory address to write
     * @param   pData    : Pointer to data to write
     * @param   Len      : Number of bytes to write
     *
     * @return  Number of bytes written
     */
    virtual int Write(uint32_t Addr, uint8_t *pData, int Len) { return SeepWrite(&vDevData, Addr, pData, Len); }

    /**
     * @brief Get EEPROM size.
     *
     * @return  Total size in bytes
     */
    uint32_t GetSize() { return vDevData.Size; }

    /**
     * @brief Get EEPROM page size
     *
     * @return  Page size in bytes
     */
    uint16_t GetPageSize() { return vDevData.PageSize; }

    /**
     * @brief	Conversion to SEEPDEV operator.
     *
     * This operator convert SEEP class to SEEPDEV to be used in C function calls
     *
     * @return	Pointer to internal SEEPDEV data.
     */
    operator SEEPDEV*  const () { return &vDevData; }

protected:

    SEEPDEV vDevData;	//!< Device address.
};

#endif

/** @} End of group Storage */

#endif	// __SEEP_H__

