/**-------------------------------------------------------------------------
@example	uart_prbs_rx.cpp


@brief	UART PRBS test

Demo code using EHAL library to do PRBS transmit test using UART

@author	Hoang Nguyen Hoan
@date	Aug. 31, 2016

@license

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

----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <chrono>
#include <time.h>

#include "uart.h"
#include "prbs.h"


char s_DevPath[] = {"/dev/cu.usbmodem142122"};

// UART configuration data
const UARTCFG g_UartCfg = {
	0,
	s_DevPath,
	static_cast<int>(strlen(s_DevPath)),
	1000000,	// Rate
	8,
	UART_PARITY_NONE,
	1,	// Stop bit
	UART_FLWCTRL_NONE,
	true,
	1, //  use APP_IRQ_PRIORITY_LOW with Softdevice
	nullptr,
	0,
	nullptr,
	0,
	nullptr,
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
