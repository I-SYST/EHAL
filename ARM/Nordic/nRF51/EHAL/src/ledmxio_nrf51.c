/*--------------------------------------------------------------------------
File   : ledmxio_nrf51.c

Author : Hoang Nguyen Hoan          Aug. 21, 2014

Desc   : This is platform specific I/O control interface for LED Matrix Control
         IDM-LMX3208 series LED matrix display

Copyright (c) 2014, I-SYST inc., all rights reserved

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

#include "nrf_gpio.h"
#include "ledmxio.h"

typedef struct {
  int  WrPin;
  int  RdPin;
  int  DataPin;
  int  EnPin;
  int  CsPins[LEDMX_MAX_ADDRPIN];
  int  NbCsPins;				// Total number of CS pins used
  LEDMX_CSTYPE CsType;
} IODEV;

IODEV g_IODev;

void LedMxIOInit(LEDMXDEV *pLedMxDev, LEDMXCFG *pCfg)
{
    IODEV *pdev = &g_IODev;
	LEDMXIOCFG *pcfg = (LEDMXIOCFG *)pCfg->pIOCfg;

    pLedMxDev->pIODev = (void *)&g_IODev;

    pdev->WrPin = pcfg->WrPin;
    nrf_gpio_cfg_output(pcfg->WrPin);
	nrf_gpio_pin_set(pcfg->WrPin);

    pdev->RdPin = pcfg->RdPin;
	nrf_gpio_cfg_output(pcfg->RdPin);
	nrf_gpio_pin_set(pcfg->RdPin);

    pdev->DataPin = pcfg->DataPin;
	nrf_gpio_cfg_output(pcfg->DataPin);

    pdev->EnPin = pcfg->EnPin;
	nrf_gpio_cfg_output(pcfg->EnPin);
	nrf_gpio_pin_set(pcfg->EnPin);

	for (int i = 0; i < LEDMX_MAX_ADDRPIN; i++)
	{
        pdev->CsPins[i] = pcfg->CsPins[i];
		if (pdev->CsPins[i] >= 0)
		{
			nrf_gpio_cfg_output(pdev->CsPins[i]);
			nrf_gpio_pin_clear(pdev->CsPins[i]);
		}
	}
	pdev->NbCsPins = pcfg->NbCsPins;
	pdev->CsType = pcfg->CsType;
}

void LedMxStartTx(LEDMXDEV *pLedMxDev, int PanelAddr)
{
	IODEV *pdev = (IODEV *)pLedMxDev->pIODev;

	nrf_gpio_pin_set(pdev->RdPin);
	nrf_gpio_pin_set(pdev->WrPin);

	if (pdev->CsType == LEDMX_CSTYPE_BIN)
	{
	      nrf_gpio_pin_set(pdev->EnPin);
	      for (int i = 0; i < pdev->NbCsPins; i++)
	      {
	          if (pdev->CsPins[i] >= 0)
	          {
	              if (PanelAddr & 1)
	                  nrf_gpio_pin_set(pdev->CsPins[i]);
	              else
	                  nrf_gpio_pin_clear(pdev->CsPins[i]);
	          }
	          PanelAddr >>= 1;
	      }
	      nrf_gpio_pin_clear(pdev->EnPin);
	}
	else
	{
	      nrf_gpio_pin_clear(pdev->CsPins[PanelAddr]);
	}
}

void LedMxTxData(LEDMXDEV *pLedMxDev, uint32_t Data, int NbBits)
{
	IODEV *pdev = (IODEV *)pLedMxDev->pIODev;
	uint32_t mask = 1 << (NbBits - 1);

	while (mask)
	{
	    nrf_gpio_pin_clear(pdev->WrPin);
	    if (Data & mask)
	    	nrf_gpio_pin_set(pdev->DataPin);
	    else
	    	nrf_gpio_pin_clear(pdev->DataPin);

	    __NOP();
	    nrf_gpio_pin_set(pdev->WrPin);

	    mask >>= 1;
	}

}

void LedMxStopTx(LEDMXDEV *pLedMxDev)
{
	IODEV *pdev = (IODEV *)pLedMxDev->pIODev;

	nrf_gpio_pin_set(pdev->WrPin);

	if (pdev->CsType == LEDMX_CSTYPE_BIN)
	{
		for (int i = 0; i < pdev->NbCsPins; i++)
		{
			if (pdev->CsPins[i] >= 0)
				nrf_gpio_pin_set(pdev->CsPins[i]);
		}
		nrf_gpio_pin_set(pdev->EnPin);
	}
	else
		nrf_gpio_pin_clear(pdev->EnPin);
}
