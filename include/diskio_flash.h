/**--------------------------------------------------------------------------
@file	diskio_flash.h

@brief	Desc   : Generic flash disk I/O driver class

Most Flash devices work in MSB bit order. This implementation only support MSB.
Make sure that the Flash is configure for MSB mode

This implementation works with most Flash devices.  There is no need to implement
for each device, just fill the config data struct and pass it to init function

Example of defining Flash device info :

-----
MX25R1635F :

static const FLASHDISKIO_CFG s_FlashDiskCfg = {
    0,
    16 * 1024 * 1024 / 8,      	// 16 Mbits
    0x10000,					// minimum erase block size
    256,						// Write page size
    3,                          // 3 bytes addressing
	NULL.						// no special init require.
    NULL,						// blocking, no wait callback
};

-----
S25FS :

static const FLASHDISKIO_CFG s_FlashDiskCfg = {
    0,
    128 * 1024 * 1024 / 8,      // 128 Mbits
    0x10000,
    512,						// Write page size
    3,
    s25fs_init,					// Special initialization require
};

bool s25fs_init(int DevNo, DeviceIntrf *pInterf)
{
    if (pInterf == NULL)
        return false;
    int cnt = 0;

    uint32_t d;

    // Enable write
    d = NFLASH_S25FS_CMD_WREN;
    cnt = pInterf->Tx(DevNo, (uint8_t*)&d, 1);

    do {
        d = NFLASH_S25FS_CMD_RDSR1;
        pInterf->StartRx(DevNo);
        cnt = pInterf->TxData((uint8_t*)&d, 1);
        cnt = pInterf->RxData((uint8_t*)&d, 1);
        pInterf->StopRx();
    } while ((d & NFLASH_S25FS_REG_SR1V_WIP));

    // Configure uniform sector arch
    uint8_t p[8];
    p[0] = NFLASH_S25FS_CMD_WRAR;
    p[1] = (NFLASH_S25FS_REG_CR3V >> 16) & 0xFF;
    p[2] = (NFLASH_S25FS_REG_CR3V >> 8) & 0xFF;
    p[3] = NFLASH_S25FS_REG_CR3V & 0xFF;
    p[4] = NFLASH_S25FS_REG_CR3NV_20h_NV | NFLASH_S25FS_REG_CR3V_02h_NV;

    cnt = pInterf->Tx(DevNo, p, 5);

    return true;
}

-----
MX66U51235F :

static const FLASHDISKIO_CFG s_FlashDiskCfg = {
    0,
    256 * 1024 * 1024 / 8,      // 256 Mbits
    0x10000,
    128,						// Write page size
    4,                          // 256+ Mbits needs 4 bytes addressing
    NULL,
    NULL
};

-----
Usage in C++ :

// SPI interface instance to be used.  Assuming it is already initialized
SPI g_Spi;

// Declare device instance
FlashDiskIO g_FlashDisk;

// Disk sector cache block in RAM
uint8_t g_FlashCacheMem[DISKIO_SECT_SIZE];
DISKIO_CACHE_DESC g_FlashCache = {
    -1, 0xFFFFFFFF, g_FlashCacheMem
};

// Initialize
g_FlashDisk.Init(s_FlashDiskCfg, &g_Spi, &g_FlashCache, 1);

// Read/Write
uint8_t buff[DISKIO_SECT_SIZE];

g_FlashDisk.SectRead(1, buff);	// Read sector 1
g_FlashDisk.SectWrite(2, buff);	// Write sector 2
g_FlashDisk.Erase();			// Masse erase flash


@author	Hoang Nguyen Hoan
@date	Aug. 30, 2016

@license

Copyright (c) 2016, I-SYST, all rights reserved
Copyright (c) 2016, Motsai, all rights reserved

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
#ifndef __DISKIO_FLASH_H__
#define __DISKIO_FLASH_H__

#include <stdint.h>
#include "diskio.h"
#include "device_intrf.h"

/** @addtogroup Storage
  * @{
  */

#define FLASH_CMD_READID			0x9F
#define FLASH_CMD_RDCR              0x15    //!< Read configuration register
#define FLASH_CMD_WRSR              0x01    //!< Write Register (Status 1, Configuration 1)
#define FLASH_CMD_WRITE             0x2
#define FLASH_CMD_READ              0x3
#define FLASH_CMD_WRDISABLE         0x4
#define FLASH_CMD_READSTATUS        0x5
#define FLASH_CMD_WRENABLE          0x6
#define FLASH_CMD_EN4B              0xB7    //!< Enable 4 bytes address
#define FLASH_CMD_EX4B              0xE9    //!< Disable 4 bytes address
#define FLASH_CMD_BLOCK_ERASE       0xD8
#define FLASH_CMD_BULK_ERASE        0xC7

#define FLASH_STATUS_WIP            (1<<0)  // Write In Progress

/**
 * @brief FlashDiskIO callback function.
 *
 * This a general callback function hook for special purpose defined in the FLASHDISKIO_CFG
 *
 * @param   DevNo 	: Device number or address used by the interface
 * @param   pInterf : Interface used to access the flash (SPI, I2C or whatever
 *
 * @return  true - Success\n
 *          false - Failed.
 */
typedef bool (*FLASHDISKIOCB)(int DevNo, DeviceIntrf * const pInterf);

typedef struct {
    int         DevNo;          //!< Device number or address for interface use
    uint64_t    TotalSize;      //!< Total Flash size in bytes
    uint32_t    EraseSize;      //!< Min erasable block size in byte
    uint32_t    WriteSize;      //!< Writable page size in bytes
    int         AddrSize;       //!< Address size in bytes
    FLASHDISKIOCB pInitCB; 		//!< For custom initialization. Set to NULL if not used
    FLASHDISKIOCB pWaitCB;		//!< If provided, this is called when there are
    							//!< long delays, such as mass erase, to allow application
    							//!< to perform other tasks while waiting
} FLASHDISKIO_CFG;


/// @brief	Flash disk base class
///
/// Most Flash devices work in MSB bit order. This implementation
/// only supports MSB mode. Make sure that the Flash is configured
/// for MSB mode.
class FlashDiskIO : public DiskIO {
public:
	FlashDiskIO();
	virtual ~FlashDiskIO() {}

	/**
	 * @brief	Initialize Flash Disk.
	 *
	 * @param	Cfg		: Flash disk configuration data
	 * @param	pInterf	: Pointer to device interface to access flash device
	 * @param	pCacheBlk	: Pointer to static cache block (optional)
	 * @param	NbCacheBlk	: Size of cache block (Number of cache sector)
	 *
	 * @return
	 * 			- true 	: Success
	 * 			- false	: Failed
	 */
	bool Init(FLASHDISKIO_CFG &Cfg, DeviceIntrf * const pInterf,
	          DISKIO_CACHE_DESC * const pCacheBlk = NULL, int NbChaceBlk = 0);

    /**
     * @brief	Get total disk size in bytes.
     *
     * @return	Total size in bytes
     */
    virtual uint64_t GetSize(void) { return vTotalSize; }

    /**
	 * @brief	Device specific minimum erasable block size in bytes.
	 *
	 * @return	Block size in bytes
	 */
	virtual uint32_t GetMinEraseSize() { return vEraseSize; }

	/**
	 * @brief	Device specific minimum write size in bytes
	 *
	 * @return	Size in bytes
	 */
	virtual uint32_t GetMinWriteSize() { return vWriteSize; }

	/**
	 * @brief	Perform mass erase (ERASE ALL).
	 *
	 * This function may take a long time to complete. If task switching is require, add delay
	 * callback function to the configuration at initialization.
	 */
	virtual void Erase();

	/**
	 * @brief	Erase Flash block.
	 *
	 * @param	BlkNo	: Starting block number to erase.
	 * @param	NbBlk	: Number of consecutive blocks to erase
	 */
	virtual void EraseBlock(uint32_t BlkNo, int NbBlk);

	/**
     * @brief	Read one sector from physical device.
     *
     * @param	SectNo	: Sector number to read
     * @param	pBuff	: Pointer to buffer to receive sector data. Must be at least
     * 					  1 sector size
     *
     * @return
     * 			- true	: Success
     * 			- false	: Failed
     */
    virtual bool SectRead(uint32_t SectNo, uint8_t *pBuff);

    /**
     * @brief	Write one sector to physical device
     *
     * @param	SectNo	: Sector number to read
     * @param	pData	: Pointer to sector data to write. Must be at least
     * 					  1 sector size
     *
     * @return
     * 			- true	: Success
     * 			- false	: Failed
     */
    virtual bool SectWrite(uint32_t SectNo, uint8_t *pData);

    /**
     * @brief	Read Flash ID
     *
     * @return	Flash ID
     */
    uint32_t ReadId();

    /**
     * @brief	Read Flash status.
     *
     * @return	Flash status
     */
    uint8_t ReadStatus();

protected:

    /**
     * @brief	Disable Flash write
     */
    void WriteDisable();

    /**
     * @brief	Enable Flash write
     *
     * @param	Timeout : Timeout counter
     *
     * @return
     * 			- true	: Success
     * 			- false	: Failed
     */
    bool WriteEnable(uint32_t Timeout = 100000);

    /**
     * @brief	Wait for Flash ready flag
     *
     * @param	Timeout : Timeout counter
     * @param	usRtyDelay	: Timeout in us before retry (optional)
     *
     * @return
     * 			- true	: Success
     * 			- false	: Failed
     */
    bool WaitReady(uint32_t Timeout = 100000, uint32_t usRtyDelay = 0);

private:
    uint32_t    vEraseSize;		//!< Min erasable block size in byte
    uint32_t    vWriteSize;		//!< Min writable size in bytes
    uint64_t    vTotalSize;		//!< Total Flash size in bytes
    int         vAddrSize;		//!< Address size in bytes
    int         vDevNo;			//!< Device No
    DeviceIntrf *vpInterf;		//!< Device interface to access Flash
    FLASHDISKIOCB vpWaitCB;		//!< User wait callback when long wait time is required. This is to allows
    							//!< user application to perform task switch or other thing while waiting.
};

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

/** @} End of group Storage */

#endif	// __DISKIO_FLASH_H__

