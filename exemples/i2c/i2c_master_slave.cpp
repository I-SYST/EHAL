/**-------------------------------------------------------------------------
@example	i2c_master_slave.cpp

@brief	This example demonstrate the use of I2C in both master and slave mode

Two I2C devices are created, one in master mode and the other in slave mode.
User is require to connected the wire to the appropriate pins.


@author	Hoang Nguyen Hoan
@date	July 21, 2018

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

#include "coredev/i2c.h"
#include "coredev/uart.h"
#include "stddev.h"
#include "board.h"

//int nRFUartEvthandler(UARTDEV *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

#define FIFOSIZE			CFIFO_MEMSIZE(512)

uint8_t g_TxBuff[FIFOSIZE];

static IOPINCFG s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		// RX
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		// TX
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		// CTS
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RTS
};

// UART configuration data
static const UARTCFG s_UartCfg = {
	0,
	s_UartPins,
	sizeof(s_UartPins) / sizeof(IOPINCFG),
	1000000,			// Rate
	8,
	UART_PARITY_NONE,
	1,					// Stop bit
	UART_FLWCTRL_NONE,
	true,
	1, 					// use APP_IRQ_PRIORITY_LOW with Softdevice
	NULL,//nRFUartEvthandler,
	true,				// fifo blocking mode
	0,
	NULL,
	FIFOSIZE,
	g_TxBuff,
};

UART g_Uart;

//********** I2C Master **********
static const I2CCFG s_I2cCfgMaster = {
	0,			// I2C device number
	{
		{0, 22, 1, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},		// SDA
		{0, 23, 1, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},	// SCL
	},
	100000,		// Rate
	I2CMODE_MASTER,
	5,			// Retry
	0,			// Number of slave addresses
	{0,},		// Slave addresses
	true,
	false,
	7,			// Interrupt prio
	NULL		// Event callback
};

I2C g_I2CMaster;

//********** I2C Slave **********

#define I2C_SLAVE_ADDR			0x22

int I2CSlaveIntrfHandler(DEVINTRF * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int BufferLen);

static const I2CCFG s_I2cCfgSlave = {
	1,			// I2C device number
	{
		{0, 24, 1, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},		// SDA
		{0, 25, 1, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},	// SCL
	},
	100000,		// Rate
	I2CMODE_SLAVE,
	5,			// Retry
	1,			// Number of slave addresses
	{I2C_SLAVE_ADDR, },	// Slave mode response addresses
	true,
	true,
	7,			// Interrupt prio
	I2CSlaveIntrfHandler	// Event callback
};

I2C g_I2CSlave;

uint8_t s_ReadRqstData[100];
uint8_t s_WriteRqstData[10];
bool s_bWriteRqst = false;
int s_Offset = 0;

int I2CSlaveIntrfHandler(DEVINTRF * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len)
{
	switch (EvtId)
	{
		case DEVINTRF_EVT_READ_RQST:
			if (s_bWriteRqst)
			{
				// There was a write command previously
				// get the offset.  parameter Len should indicates number of byte
				// was written prior to change state to read request
				if (Len > 0)
				{
					s_Offset = s_WriteRqstData[0];
				}
				s_bWriteRqst = false;
			}
			g_I2CSlave.SetReadRqstData(0, &s_ReadRqstData[s_Offset], 10);

			break;

		case DEVINTRF_EVT_WRITE_RQST:
			s_bWriteRqst = true;
			break;

		case DEVINTRF_EVT_COMPLETED:

			if (s_bWriteRqst == false)
			{
				// No previous write command. This means a continuous read
				// move to last read
				s_Offset += Len;
				if (s_Offset >= 100)
				{
					// Reset offset if we moved beyond max buffer
					s_Offset = 0;
				}
			}
			s_bWriteRqst = false;
			break;
	}
}

void HardwareInit()
{
	g_Uart.Init(s_UartCfg);
	UARTRetargetEnable(g_Uart, STDIN_FILENO);
	UARTRetargetEnable(g_Uart, STDOUT_FILENO);

	printf("Init I2C Master/Slave demo\r\n");
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
	uint8_t reg;
	uint8_t buff[10];

	HardwareInit();

	g_I2CMaster.Init(s_I2cCfgMaster);

	g_I2CSlave.Init(s_I2cCfgSlave);

	// Fill dummy data for debugging and validation
	for (int i = 0; i < 100; i++)
	{
		s_ReadRqstData[i] = i;
	}

	memset(s_WriteRqstData, 0xff, 10);

	// Set pointer to data to be send upon receive of a read request
	g_I2CSlave.SetReadRqstData(0, s_ReadRqstData, 10);

	// Set pointer to buffer to receive from write request
	g_I2CSlave.SetWriteRqstBuffer(0, s_WriteRqstData, 2);

	memset(buff, 0xFF, 10);

	reg = 15; // want to read from offset 15

	// Master send read command to read 10 bytes from offset defined in data[0]
	g_I2CMaster.Read(I2C_SLAVE_ADDR, &reg, 1, buff, 10);

	// Master send read command without setting anything
	g_I2CMaster.Read(I2C_SLAVE_ADDR, NULL, 0, buff, 5);

	while (1)
	{
		__WFE();
	}

	return 0;
}
