/*--------------------------------------------------------------------------
File   : main.cpp

Author : Hoang Nguyen Hoan          Aug. 31, 2016

Desc   : UART PRBS test
		 Demo code using EHAL library to do PRBS transmit test using UART

Copyright (c) 2016, I-SYST inc., all rights reserved

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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <chrono>
#include <time.h>

#include "coredev/uart.h"
#include "prbs.h"


char s_DevPath[] = {"/dev/cu.usbmodem00304160000942"};

// UART configuration data
const UARTCFG g_UartCfg = {
    .DevNo = 0,
    .pIoMap = s_DevPath,
    .IoMapLen = static_cast<int>(strlen(s_DevPath)),
    .Rate = 1000000,            // Rate
    .DataBits = 8,
    .Parity = UART_PARITY_NONE,
    .StopBits = 1,                    // Stop bit
    .FlowControl = UART_FLWCTRL_NONE,
    .bIntMode = true,
    .IntPrio = 1,                     // use APP_IRQ_PRIORITY_LOW with Softdevice
    .EvtCallback = nullptr,
    .bFifoBlocking = true,                // fifo blocking mode
    .RxMemSize = 0,
    .pRxMem = NULL,
    .TxMemSize = 0,//FIFOSIZE,
    .pTxMem = NULL,//g_TxBuff,
    .bDMAMode = true,
};

#define DEMO_C
#ifdef DEMO_C
// For C
UARTDEV g_UartDev;
#else
// For C++
// UART object instance
UART g_Uart;
#endif


//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.g
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
	bool res;

#ifdef DEMO_C
	res = UARTInit(&g_UartDev, &g_UartCfg);
#else
	res = g_Uart.Init(g_UartCfg);
#endif

	uint8_t d = 0xff;
    uint8_t val = 0;
    uint32_t errcnt = 0;
    uint32_t cnt = 0;
    auto t_start = std::chrono::high_resolution_clock::now();
    auto t_end = std::chrono::high_resolution_clock::now();
    
    std::chrono::duration<float> elapse = std::chrono::duration<float>(0);
    t_start = std::chrono::high_resolution_clock::now();

    time_t t;
    double e = 0.0;
    bool isOK = false;
//    do {
#ifdef DEMO_C
    while (UARTRx(&g_UartDev, &d, 1) <= 0);
#else
    while (g_Uart.Rx(&d, 1) <= 0);
#endif
    	if (val == d)
            isOK = true;
    	val = Prbs8(d);
 //   } while (!isOK);
    
	while(1)
	{
       // t_start = std::chrono::high_resolution_clock::now();
        t = time(NULL);
#ifdef DEMO_C
        while (UARTRx(&g_UartDev, &d, 1) <= 0);
#else
        while (g_Uart.Rx(&d, 1) <= 0);
#endif
		{
            e += difftime(time(NULL), t);
          //  t_end = std::chrono::high_resolution_clock::now();
            //elapse += std::chrono::duration<float>(t_end-t_start);
            cnt++;
            
			// If success send next code
           // printf("%x ", d);
            if (val != d)
            {
                errcnt++;
               // printf("PRBS %u errors %x %x\n", errcnt, val, d);
            }
            else if ((cnt & 0x7fff) == 0)
            {
                printf("PRBS rate %.3f B/s, err : %u\n", cnt / e, errcnt);
//                printf("PRBS rate %.3f B/s, err : %u\n", cnt / elapse.count(), errcnt);

            }
			val = Prbs8(d);
		}
	}
	return 0;
}
