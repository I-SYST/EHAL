/**--------------------------------------------------------------------------
@file	diskio_flash.h

@brief	Generic flash disk I/O driver class

Most Flash devices work in MSB bit order. This implementation only support MSB.
Make sure that the Flash is configure for MSB mode

This implementation works with most Flash devices.  There is no need to implement
for each device, just fill the config data struct and pass it to init function

Example of defining Flash device info :

-----
MX25R1635F :

static const FLASHDISKIO_CFG s_FlashDiskCfg = {
    .DevNo = 0,
    .TotalSize = 16 * 1024 / 8,		// 16 Mbits in KBytes
	.SectSize = 4,					// 4K
    .BlkSize = 64,					// 64K
    .WriteSize = 256,				// Write page size
    .AddrSize = 3,              	// 3 bytes addressing
	.pInitCB = NULL.				// no special init require.
    .pWaitCB = NULL,				// blocking, no wait callback
};

-----
S25FS :

static const FLASHDISKIO_CFG s_FlashDiskCfg = {
    .DevNo = 0,
    .TotalSize = 128 * 1024 / 8,	// 128 Mbits
	.SectSize = 4,					// 4K
    .BlkSize = 64,					// 64K
    .WriteSize = 512,				// Write page size
    .AddrSize = 3,
    .pInitCB = s25fs_init,			// Special initialization require
    .pWaitCB = NULL,
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
    .DevNo = 0,
    .TotalSize = 256 * 1024 / 8,	// 256 Mbits in KBytes
	.SectSize = 4,					// 4K
    .BlkSize = 64,					// 64K
    .WriteSize = 128,				// Write page size
    .AddrSize = 4,                  // 256+ Mbits needs 4 bytes addressing
    .pInitCB = NULL,
    .pWaitCB = NULL
};


// Quad SPI Flash Micron N25Q128A
static FLASHDISKIO_CFG s_N25Q128A_QFlashCfg = {
    .DevNo = 0,
    .TotalSize = 128 * 1024 / 8,	// 128 Mbits in KBytes
	.SectSize = 4,
    .BlkSize = 32,
    .WriteSize = 256,
    .AddrSize = 3,					// 3 bytes addressing
    .pInitCB = NULL,//MX25U1635E_init,
    .pWaitCB = NULL,//FlashWriteDelayCallback,
	.RdCmd = { FLASH_CMD_QREAD, 10},
	.WrCmd = { FLASH_CMD_QWRITE, 0 },
};

// Quad SPI Flash Macronix MX25R3235F
static FLASHDISKIO_CFG s_MX25R3235F_QFlashCfg = {
    .DevNo = 0,
    .TotalSize = 32 * 1024 / 8,		// 16 Mbits
	.SectSize = 4,
    .BlkSize = 64,
    .WriteSize = 256,
    .AddrSize = 3,					// 3 bytes addressing
    .pInitCB = NULL,//MX25U1635E_init,
    .pWaitCB = NULL,//FlashWriteDelayCallback,
	.RdCmd = { FLASH_CMD_4READ, 6},
	.WrCmd = { FLASH_CMD_4WRITE, 0 },
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
g_FlashDisk.Erase();			// Mass erase flash


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
#ifndef __DISKIO_FLASH_H__
#define __DISKIO_FLASH_H__

#include <stdint.h>
#include <string.h>

#include "diskio.h"
#include "device_intrf.h"

/** @addtogroup Storage
  * @{
  */

#define FLASH_CMD_READID			0x9F
#define FLASH_CMD_RDCR              0x15    //!< Read configuration register
#define FLASH_CMD_WRSR              0x01    //!< Write Register (Status 1, Configuration 1)
#define FLASH_CMD_WRITE             0x2
#define FLASH_CMD_4WRITE            0x38
#define FLASH_CMD_QWRITE			0x32
#define FLASH_CMD_E4WRITE			0x12
#define FLASH_CMD_READ              0x3
#define FLASH_CMD_FAST_READ			0xB
#define FLASH_CMD_DREAD				0x3B	//!< Dual read - address single, data dual
#define FLASH_CMD_QREAD				0x6B	//!< Quad read - address single, data quad
#define FLASH_CMD_2READ				0xBB	//!< 2 x I/O read - address dual, data dual
#define FLASH_CMD_4READ				0xEB	//!< 4 x I/O read - address quad, data quad
#define FLASH_CMD_WRDISABLE         0x4
#define FLASH_CMD_READSTATUS        0x5
#define FLASH_CMD_WRENABLE          0x6
#define FLASH_CMD_EN4B              0xB7    //!< Enable 4 bytes address
#define FLASH_CMD_EX4B              0xE9    //!< Disable 4 bytes address
#define FLASH_CMD_SECTOR_ERASE		0x20	//!< Sector erase
#define FLASH_CMD_BLOCK_ERASE_32    0x52	//!< 32KB Block erase
#define FLASH_CMD_BLOCK_ERASE       0xD8	//!< Block erase
#define FLASH_CMD_BULK_ERASE        0xC7	//!< Chip erase
#define FLASH_CMD_BULK_ERASE_ALT	0x60	//!< Alternate chip erase command

#define FLASH_STATUS_WIP            (1<<0)  // Write In Progress

#pragma pack(push, 1)
/// Quad SPI flash can have different command code and dummy cycle.
/// This structure is to define supported command for the Flash config.
/// Assign the appropriate command code and dymmy cycle count in the config structure
typedef struct __Quad_Flash_Cmd {
	uint8_t Cmd;				//!< Command code
	uint8_t DummyCycle;			//!< Dummy cycle
} CMDCYCLE;

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
    uint32_t    TotalSize;      //!< Total Flash size in KBytes
    uint16_t    SectSize;		//!< Sector erase size in KBytes
    uint16_t	BlkSize;		//!< Block erase size in KBytes
    uint32_t    WriteSize;      //!< Writable page size in bytes
    int         AddrSize;       //!< Address size in bytes
    uint32_t	DevId;			//!< Device ID, read using FLASH_CMD_READID
    int			DevIdSize;		//!< Length of device id in bytes to read (max 4 bytes)
    FLASHDISKIOCB pInitCB; 		//!< For custom initialization. Set to NULL if not used
    FLASHDISKIOCB pWaitCB;		//!< If provided, this is called when there are
    							//!< long delays, such as mass erase, to allow application
    							//!< to perform other tasks while waiting
    CMDCYCLE	RdCmd;			//!< QSPI read cmd and dummy cycle
    CMDCYCLE	WrCmd;			//!< QSPI write cmd and dummy cycle
} FLASHDISKIO_CFG;

#pragma pack(pop)

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
	bool Init(const FLASHDISKIO_CFG &Cfg, DeviceIntrf * const pInterf,
	          DISKIO_CACHE_DESC * const pCacheBlk = NULL, int NbCacheBlk = 0);

    /**
     * @brief	Get total disk size in bytes.
     *
     * @return	Total size in KBytes
     */
    virtual uint32_t GetSize(void) { return vTotalSize; }

    /**
	 * @brief	Device specific minimum erasable block size in bytes.
	 *
	 * @return	Block size in bytes
	 */
	virtual uint32_t GetMinEraseSize() { return vSectSize; }

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
	 * @brief	Erase Flash block.
	 *
	 * @param	BlkNo	: Starting block number to erase.
	 * @param	NbBlk	: Number of consecutive blocks to erase
	 */
	virtual void EraseSector(uint32_t SectNo, int NbSect);

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
     * @param	Len : Length of id to read in bytes
     *
     * @return	Flash ID
     */
    uint32_t ReadId(int Len);

    /**
     * @brief	Read Flash status.
     *
     * @return	Flash status
     */
    uint8_t ReadStatus();

    /**
     * @brief	Get the sector erase size
     *
     * The return value is normally set via configuration structure at init
     *
     * @return	Size in KBytes
     */
    uint16_t SectEraseSize() { return vSectSize; }

    /**
     * @brief	Get the block erase size
     *
     * The return value is normally set via configuration structure at init
     *
     * @return	Size in KBytes
     */
    uint16_t BlockEraseSize() { return vBlkSize; }


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
    uint16_t    vSectSize;		//!< Erasable sector size in KBytes
    uint16_t    vBlkSize;		//!< Erasable block size in KBytes
    uint32_t    vWriteSize;		//!< Min writable size in bytes
    uint32_t    vTotalSize;		//!< Total Flash size in KBytes
    int         vAddrSize;		//!< Address size in bytes
    int         vDevNo;			//!< Device No
    DeviceIntrf *vpInterf;		//!< Device interface to access Flash
    FLASHDISKIOCB vpWaitCB;		//!< User wait callback when long wait time is required. This is to allows
    							//!< user application to perform task switch or other thing while waiting.
    CMDCYCLE	vRdCmd;			//!< QSPI read/write and dummy cycle
    CMDCYCLE	vWrCmd;			//!< QSPI read/write and dummy cycle
};

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

/** @} End of group Storage */

#endif	// __DISKIO_FLASH_H__

