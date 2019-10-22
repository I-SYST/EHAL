/**-------------------------------------------------------------------------
@example	flash_memory_demo.cpp

@brief	Example code using SPI Flash memory

@author	Hoang Nguyen Hoan
@date	Mars 8, 2019

@license

Copyright (c) 2019, I-SYST, all rights reserved

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
#include <stdio.h>

// EHAL includes
#include "coredev/uart.h"
#include "coredev/spi.h"
#include "diskio_flash.h"
#include "stddev.h"
#include "board.h"
#include "idelay.h"

#define FIFOSIZE			CFIFO_MEMSIZE(256)

uint8_t g_UarTxBuff[FIFOSIZE];

// Assign UART pins
static IOPINCFG s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RX
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// TX
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// CTS
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},// RTS
};

// UART configuration data
static const UARTCFG s_UartCfg = {
	.DevNo = 0,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPINCFG),
	.Rate = 1000000,			// Rate
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,					// Stop bit
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = 1, 					// use APP_IRQ_PRIORITY_LOW with Softdevice
	.EvtCallback = NULL,//nRFUartEvthandler,
	.bFifoBlocking = true,				// fifo blocking mode
	.RxMemSize = 0,
	.pRxMem = NULL,
	.TxMemSize = FIFOSIZE,
	.pTxMem = g_UarTxBuff,
};

UART g_Uart;

//********** SPI Master **********
static const IOPINCFG s_SpiPins[] = {
    {SPI_SCK_PORT, SPI_SCK_PIN, SPI_SCK_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		// SCK
    {SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// MISO
    {SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		// MOSI
    {SPI_FLASH_CS_PORT, SPI_FLASH_CS_PIN, SPI_FLASH_CS_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// CS
};

static const SPICFG s_SpiCfg = {
	.DevNo = SPI_DEVNO,
	.Type = SPITYPE_NORMAL,
    .Mode = SPIMODE_MASTER,
	.pIOPinMap = s_SpiPins,
    .NbIOPins = sizeof( s_SpiPins ) / sizeof( IOPINCFG ),
    .Rate = 4000000,   // Speed in Hz
    .DataSize = 8,      // Data Size
    .MaxRetry = 5,      // Max retries
    .BitOrder = SPIDATABIT_MSB,
    .DataPhase = SPIDATAPHASE_FIRST_CLK, // Data phase
    .ClkPol = SPICLKPOL_HIGH,         // clock polarity
    .ChipSel = SPICSEL_AUTO,
	.bDmaEn = true,	// DMA
	.bIntEn = false,
    .IntPrio = 6, //APP_IRQ_PRIORITY_LOW,      // Interrupt priority
    .EvtCB = NULL
};

SPI g_Spi;

FlashDiskIO g_FlashDiskIO;

static uint8_t s_FlashCacheMem[DISKIO_SECT_SIZE];
DISKIO_CACHE_DESC g_FlashCache = {
    -1, 0xFFFFFFFF, s_FlashCacheMem
};

/// mx66u51235f
bool mx66u51235f_init(int pDevNo, DeviceIntrf* ppInterface);
bool FlashWriteDelayCallback(int DevNo, DeviceIntrf *pInterf)
{
	return true;
}

static FLASHDISKIO_CFG s_FlashDiskCfg = {
    .DevNo = 0,
    .TotalSize = 256 * 1024 * 1024 / 8,      // 256 Mbits
    .EraseSize = 0x10000,
    .WriteSize = 128,
    .AddrSize = 4,                          // 256+ Mbits needs 4 bytes addressing
    .pInitCB = mx66u51235f_init,
    .pWaitCB = FlashWriteDelayCallback,
};

bool mx66u51235f_init(int pDevNo, DeviceIntrf* ppInterface)
{
    if (ppInterface == NULL)
        return false;

    int cnt = 0;

    uint32_t d;
    uint32_t r = 0;

    d = FLASH_CMD_READID;
    cnt = ppInterface->Read(pDevNo, (uint8_t*)&d, 1, (uint8_t*)&r, 2 );
    if ( r != 0x25C2 )
    	return false;

    // Enable write
    d = FLASH_CMD_EN4B;
    cnt = ppInterface->Tx(pDevNo, (uint8_t*)&d, 1);

    return true;
}

//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.
//
// For example, for toolchains derived from GNU Tools for Embedded,
// to enable semi-hosting, the following was added to the linker:
//
// --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lc -lm -lrdimon -Wl,--end-group
//
// Adjust it for other toolchains.
//

int main()
{
	g_Uart.Init(s_UartCfg);

	// Retarget printf to UART
	UARTRetargetEnable(g_Uart, STDOUT_FILENO);
	UARTRetargetEnable(g_Uart, STDIN_FILENO);

	printf("Flash Memory Demo\r\n");
	//getchar();

	g_Spi.Init(s_SpiCfg);

    IOPinConfig(FLASH_HOLD_PORT, FLASH_HOLD_PIN, FLASH_HOLD_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL);

	g_FlashDiskIO.Init(s_FlashDiskCfg, &g_Spi, &g_FlashCache, 1);

	uint8_t buff[512];
	uint8_t tmp[512];
	uint16_t *p = (uint16_t*)buff;

	memset(tmp, 0, 512);
	for (int i = 0; i < 256; i++)
	{
		p[i] = i;
	}


	printf("Erasing... Please wait\r\n");

	// Ease could take a few minutes
	g_FlashDiskIO.Erase();

	printf("Writing 2KB data...\r\n");

	g_FlashDiskIO.SectWrite(0, buff);
	g_FlashDiskIO.SectWrite(2, buff);
	g_FlashDiskIO.SectWrite(4, buff);
	g_FlashDiskIO.SectWrite(8, buff);

	printf("Validate readback...\r\n");

	g_FlashDiskIO.SectRead(0, tmp);

	if (memcmp(buff, tmp, 512) != 0)
	{
		printf("Sector 0 verify failed\r\n");
	}
	else
	{
		printf("Sector 0 verify success\r\n");
	}

	memset(tmp, 0, 512);
	g_FlashDiskIO.SectRead(2, tmp);
	if (memcmp(buff, tmp, 512) != 0)
	{
		printf("Sector 2 verify failed\r\n");
	}
	else
	{
		printf("Sector 2 verify success\r\n");
	}

	memset(tmp, 0, 512);
	g_FlashDiskIO.SectRead(4, tmp);
	if (memcmp(buff, tmp, 512) != 0)
	{
		printf("Sector 4 verify failed\r\n");
	}
	else
	{
		printf("Sector 4 verify success\r\n");
	}

	memset(tmp, 0, 512);
	g_FlashDiskIO.SectRead(8, tmp);
	if (memcmp(buff, tmp, 512) != 0)
	{
		printf("Sector 8 verify failed\r\n");
	}
	else
	{
		printf("Sector 8 verify success\r\n");
	}

}
