/*--------------------------------------------------------------------------
File   : adc_nrf52_saadc.cpp

Author : Hoang Nguyen Hoan          June 16, 2017

Desc   : ADC implementation for Nordic nRF52 SAADC.
         Nordic SCAN mode does work with timer.
         Nordic Continuous mode work with timer but only 1 channel
         Therefore this implementation doesn't use Nordic continous mode
         Single mode is without interrupt, start conversion must be called
         at each conversion
         if interrupt is use, continuous sample is used.  Result is queued
         in a CFIFO

         Currently sample count is return as time stamp.

Copyright (c) 2017, I-SYST inc., all rights reserved

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
#include <stdint.h>

#include "adc_nrf52_saadc.h"

typedef struct __ADC_nRF52_Data {
	AdcnRF52 *pDevObj;
	ADC_EVTCB EvtHandler;
	int NbChanAct;
    int SampleCnt;
	uint16_t ChanState[SAADC_NRF52_MAX_CHAN];
	uint8_t	RefVoltIdx[SAADC_NRF52_MAX_CHAN];
	float GainFactor[SAADC_NRF52_MAX_CHAN];		// Pre-calculated factor
	uint32_t Period;
    int16_t ResData[SAADC_NRF52_MAX_CHAN];
    HCFIFO	hFifo[SAADC_NRF52_MAX_CHAN];

} ADCNRF52_DATA;

static ADCNRF52_DATA s_AdcnRF52DevData = {
	NULL
};

//#define ADC_CFIFO_SIZE		CFIFO_TOTAL_MEMSIZE(SAADC_NRF52_MAX_CHAN, sizeof(ADC_DATA))

//static uint8_t s_ADCnRF52FifoMem[ADC_CFIFO_SIZE];

extern "C" void SAADC_IRQHandler()
{
	ADC_EVT evt = ADC_EVT_UNKNOWN;

	if (NRF_SAADC->EVENTS_STARTED)
	{
		NRF_SAADC->EVENTS_STARTED = 0;
        NRF_SAADC->TASKS_SAMPLE = 1;
	}
	if (NRF_SAADC->EVENTS_CALIBRATEDONE)
	{
	    NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
	    NRF_SAADC->EVENTS_RESULTDONE = 0;
	    NRF_SAADC->EVENTS_DONE = 0;
	    NRF_SAADC->EVENTS_END = 0;
	}
	if (NRF_SAADC->EVENTS_END)
	{
		s_AdcnRF52DevData.SampleCnt++;

		if (s_AdcnRF52DevData.pDevObj)
		{
			int cnt = 0;
			int timeout = 1000000;

			while (NRF_SAADC->RESULT.AMOUNT == 0 && timeout-- > 0);

			for (int i = 0; i < SAADC_NRF52_MAX_CHAN && cnt < NRF_SAADC->RESULT.AMOUNT; i++)
			{
				if (s_AdcnRF52DevData.ChanState[i] != 0)
				{
					if (s_AdcnRF52DevData.hFifo[i] != NULL)
					{
						ADC_DATA *p = (ADC_DATA*)CFifoPut(s_AdcnRF52DevData.hFifo[i]);
						if (p == NULL)
							break;

						p->Chan = i;
						//
						// *** Factor calculation
						// Vin = ADCresult * Reference / (Resolution * Gain)
						// => GainFactor = Reference / (Resolution * Gain)
						// => Vin = ADCresult * GainFactor
						p->Data = (float)s_AdcnRF52DevData.ResData[i] * s_AdcnRF52DevData.GainFactor[i];
						p->Timestamp = s_AdcnRF52DevData.SampleCnt;
						cnt++;
					}
				}
			}

	        evt = ADC_EVT_DATA_READY;

			s_AdcnRF52DevData.EvtHandler(s_AdcnRF52DevData.pDevObj, evt);
		}

        NRF_SAADC->RESULT.AMOUNT = 0;
        NRF_SAADC->EVENTS_RESULTDONE = 0;
        NRF_SAADC->EVENTS_DONE = 0;
        NRF_SAADC->EVENTS_END = 0;

        if (s_AdcnRF52DevData.pDevObj->Mode() == ADC_CONV_MODE_CONTINUOUS)
        	NRF_SAADC->TASKS_START = 1;
	}
	if (NRF_SAADC->EVENTS_DONE)
	{
	    NRF_SAADC->EVENTS_DONE = 0;
        NRF_SAADC->EVENTS_RESULTDONE = 0;
	}
	NVIC_ClearPendingIRQ(SAADC_IRQn);
}

bool nRF52ADCWaitForEnd(int32_t Timeout)
{
	do {
		if (NRF_SAADC->EVENTS_END != 0)
		{
			NRF_SAADC->EVENTS_END = 0;
			return true;
		}
	} while (--Timeout > 0);

	return false;
}

bool nRF52ADCWaitBusy(int32_t Timeout)
{
	do {
		if (NRF_SAADC->STATUS == 0)
			return true;
	} while (--Timeout > 0);

	return false;
}

bool nRF52ADCWaitForStop(int32_t Timeout)
{
	do {
		if (NRF_SAADC->EVENTS_STOPPED == 1)
		{
			NRF_SAADC->EVENTS_STOPPED = 0;

			return true;
		}
	} while (--Timeout > 0);

	return false;
}

AdcnRF52::AdcnRF52()
{
	memset(&s_AdcnRF52DevData, 0, sizeof(s_AdcnRF52DevData));
}

AdcnRF52::~AdcnRF52()
{
	s_AdcnRF52DevData.pDevObj = NULL;
}

/**
 * @brief	Execute auto calibration
 *
 * @return	true - success
 */
bool AdcnRF52::Calibrate()
{
	nRF52ADCWaitBusy(1000000);

	NRF_SAADC->TASKS_STOP = 1;

	nRF52ADCWaitForStop(100000);

	NRF_SAADC->RESULT.PTR = 0;
	NRF_SAADC->RESULT.MAXCNT = 0;
	NRF_SAADC->RESULT.AMOUNT = 0;

	NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
	NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;

	if (vbInterrupt == false)
	{
		int32_t timeout = 1000000;

	    do {
            if (NRF_SAADC->EVENTS_CALIBRATEDONE != 0)
            {
                NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
                NRF_SAADC->EVENTS_RESULTDONE = 0;
                NRF_SAADC->EVENTS_DONE = 0;
                NRF_SAADC->EVENTS_END = 0;
                NRF_SAADC->EVENTS_STARTED = 0;
                NRF_SAADC->RESULT.PTR = 0;
            	NRF_SAADC->RESULT.MAXCNT = 0;
            	NRF_SAADC->RESULT.AMOUNT = 0;

                return true;
            }
        } while (timeout-- > 0);
	}

	return false;
}

bool AdcnRF52::Init(const ADC_CFG &Cfg, Timer *pTimer, DeviceIntrf *pIntrf)
{
	if (s_AdcnRF52DevData.pDevObj != NULL && s_AdcnRF52DevData.pDevObj != this)
		return false;

	memset(&s_AdcnRF52DevData, 0, sizeof(s_AdcnRF52DevData));

	// Stop current process
    NRF_SAADC->TASKS_STOP = 1;
    nRF52ADCWaitForStop(10000);

    // Flush all interrupt
    NRF_SAADC->INTENCLR = 0xFFFFFFFF;
    NVIC_ClearPendingIRQ(SAADC_IRQn);

    NRF_SAADC->RESULT.PTR = 0;
    NRF_SAADC->RESULT.MAXCNT = 0;
    NRF_SAADC->RESULT.AMOUNT = 0;

    // Clear all events
    NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
    NRF_SAADC->EVENTS_DONE = 0;
    NRF_SAADC->EVENTS_RESULTDONE = 0;
	NRF_SAADC->EVENTS_END = 0;
	NRF_SAADC->EVENTS_STARTED = 0;
	NRF_SAADC->EVENTS_STOPPED = 0;

	Resolution(Cfg.Resolution);

	int ov = 31 - __builtin_clzl(Cfg.OvrSample);

	if (ov > 0)
	{
	    NRF_SAADC->OVERSAMPLE = (ov << SAADC_OVERSAMPLE_OVERSAMPLE_Pos) & SAADC_OVERSAMPLE_OVERSAMPLE_Msk;
	}
	else
	{
		// none
		NRF_SAADC->OVERSAMPLE = SAADC_OVERSAMPLE_OVERSAMPLE_Bypass << SAADC_OVERSAMPLE_OVERSAMPLE_Pos;
	}

	vMode = Cfg.Mode;
	if (Cfg.Mode == ADC_CONV_MODE_CONTINUOUS)
	{
		Rate(Cfg.Rate);
	}
	else
	{
		vRate = 0;
		s_AdcnRF52DevData.Period = 0;
		NRF_SAADC->SAMPLERATE = (SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos);
	}

	vbInterrupt = Cfg.bInterrupt;

	s_AdcnRF52DevData.pDevObj = this;

	SetRefVoltage(Cfg.pRefVolt, Cfg.NbRefVolt);
	SetEvtHandler(Cfg.EvtHandler);
	s_AdcnRF52DevData.EvtHandler = Cfg.EvtHandler;

	NVIC_ClearPendingIRQ(SAADC_IRQn);

	if (Cfg.bInterrupt == true)
	{
		vIntPrio = Cfg.IntPrio;

		NRF_SAADC->INTENSET = (1 << SAADC_INTENSET_END_Pos)
							  | (1 << SAADC_INTENSET_DONE_Pos)
							  | (1 << SAADC_INTENSET_STARTED_Pos);

		NVIC_SetPriority(SAADC_IRQn, Cfg.IntPrio);
		NVIC_EnableIRQ(SAADC_IRQn);
	}

	NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos);

	//Calibrate();

	return true;
}

bool AdcnRF52::Enable()
{
    NRF_SAADC->RESULT.PTR = 0;
    NRF_SAADC->RESULT.MAXCNT = 0;
    NRF_SAADC->RESULT.AMOUNT = 0;

    NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
    NRF_SAADC->EVENTS_RESULTDONE = 0;
    NRF_SAADC->EVENTS_END = 0;
	NRF_SAADC->EVENTS_STARTED = 0;
	NRF_SAADC->EVENTS_STOPPED = 0;

	NVIC_ClearPendingIRQ(SAADC_IRQn);

	if (vbInterrupt == true)
	{
		NRF_SAADC->INTENSET = (1 << SAADC_INTENSET_END_Pos)
							  | (1 << SAADC_INTENSET_DONE_Pos)
							  | (1 << SAADC_INTENSET_STARTED_Pos);

		NVIC_SetPriority(SAADC_IRQn, vIntPrio);
		NVIC_EnableIRQ(SAADC_IRQn);
	}

	s_AdcnRF52DevData.SampleCnt = 0;
	NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos);

	return true;
}

void AdcnRF52::Disable()
{
    NRF_SAADC->TASKS_STOP = 1;
    nRF52ADCWaitForStop(10000);

    // Flush all interrupt
    NRF_SAADC->INTENCLR = 0xFFFFFFFF;
    NVIC_ClearPendingIRQ(SAADC_IRQn);
	NVIC_DisableIRQ(SAADC_IRQn);

    NRF_SAADC->RESULT.PTR = 0;
    NRF_SAADC->RESULT.MAXCNT = 0;
    NRF_SAADC->RESULT.AMOUNT = 0;

    // Clear all events
    NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
    NRF_SAADC->EVENTS_DONE = 0;
    NRF_SAADC->EVENTS_RESULTDONE = 0;
	NRF_SAADC->EVENTS_END = 0;
	NRF_SAADC->EVENTS_STARTED = 0;
	NRF_SAADC->EVENTS_STOPPED = 0;

	// Disconnect all channels
	for (int i = 0; i < SAADC_NRF52_MAX_CHAN; i++)
	{
		NRF_SAADC->CH[i].PSELP = 0;
		NRF_SAADC->CH[i].PSELN = 0;
	}

	NRF_SAADC->ENABLE = 0;
}

void AdcnRF52::Reset()
{
	StopConversion();

	Disable();
	Enable();
}

uint32_t AdcnRF52::Rate(uint32_t Val)
{
	if (vMode == ADC_CONV_MODE_CONTINUOUS)
	{
	    // TODO: this has no effect.  Need real clock.
		int cc = 2047;
		if (Val > 0)
			cc = (16000000 / Val);
		if (cc > 2047)
			cc = 2047;
		if (cc < 80)
			cc = 80;
		vRate = 16000000 / cc;

		s_AdcnRF52DevData.Period = (1000000 + (vRate >> 1))/ vRate;

		NRF_SAADC->SAMPLERATE = (SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos)
								| ((cc  << SAADC_SAMPLERATE_CC_Pos) & SAADC_SAMPLERATE_CC_Msk);
	}

	return vRate;
}

uint16_t AdcnRF52::Resolution(uint16_t Val)
{
	if (Val < 10)
	{
		// 8 bits
		vResolution = 8;
		NRF_SAADC->RESOLUTION = (SAADC_RESOLUTION_VAL_8bit << SAADC_RESOLUTION_VAL_Pos);
	}
	else if (Val < 12)
	{
		// 10 bits
		vResolution = 10;
		NRF_SAADC->RESOLUTION = (SAADC_RESOLUTION_VAL_10bit << SAADC_RESOLUTION_VAL_Pos);
	}
	else if (Val < 14)
	{
		// 12 bits
		vResolution = 12;
		NRF_SAADC->RESOLUTION = (SAADC_RESOLUTION_VAL_12bit << SAADC_RESOLUTION_VAL_Pos);
	}
	else
	{
		// 14 bits
		vResolution = 14;
		NRF_SAADC->RESOLUTION = (SAADC_RESOLUTION_VAL_14bit << SAADC_RESOLUTION_VAL_Pos);
	}

	return vResolution;
}

bool AdcnRF52::OpenChannel(const ADC_CHAN_CFG *pChanCfg, int NbChan)
{
	if (pChanCfg == NULL || NbChan == 0)
		return false;

	NRF_SAADC->ENABLE = 0;

	for (int i = 0; i < NbChan; i++)
	{
		uint32_t chconfig = 0;	// CH[].CONFIG register value

		if (pChanCfg[i].pFifoMem != NULL && pChanCfg[i].FifoMemSize > CFIFO_TOTAL_MEMSIZE(2, sizeof(ADC_DATA)))
		{
			s_AdcnRF52DevData.hFifo[i] = CFifoInit(pChanCfg[i].pFifoMem, pChanCfg[i].FifoMemSize, sizeof(ADC_DATA), false);
		}
		else
		{
			s_AdcnRF52DevData.hFifo[i] = NULL;
		}

		NRF_SAADC->CH[pChanCfg[i].Chan].PSELP = pChanCfg[i].PinP.PinNo + 1;
		s_AdcnRF52DevData.ChanState[pChanCfg[i].Chan] = (pChanCfg[i].PinP.PinNo + 1) & 0xFF;

		switch (pChanCfg[i].PinP.Conn)
		{
			case ADC_PIN_CONN_PULLUP:
				chconfig |= (SAADC_CH_CONFIG_RESP_Pullup << SAADC_CH_CONFIG_RESP_Pos);
				break;
			case ADC_PIN_CONN_PULLDOWN:
				chconfig |= (SAADC_CH_CONFIG_RESP_Pulldown << SAADC_CH_CONFIG_RESP_Pos);
				break;
			case ADC_PIN_CONN_VDD:
				chconfig |= (SAADC_CH_CONFIG_RESP_VDD1_2 << SAADC_CH_CONFIG_RESP_Pos);
				break;
		}

		uint32_t resdiv;

		if (pChanCfg[i].Type == ADC_CHAN_TYPE_DIFFERENTIAL)
		{
			resdiv = 1 << (vResolution - 1);
			NRF_SAADC->CH[pChanCfg[i].Chan].PSELN = pChanCfg[i].PinN.PinNo + 1;

			// In case of differential, Negative channel is kept
			s_AdcnRF52DevData.ChanState[pChanCfg[i].Chan] |= (pChanCfg[i].PinN.PinNo + 1) << 8;

			chconfig |= SAADC_CH_CONFIG_MODE_Diff << SAADC_CH_CONFIG_MODE_Pos;

			switch (pChanCfg[i].PinN.Conn)
			{
				case ADC_PIN_CONN_PULLUP:
					chconfig |= (SAADC_CH_CONFIG_RESN_Pullup << SAADC_CH_CONFIG_RESN_Pos);
					break;
				case ADC_PIN_CONN_PULLDOWN:
					chconfig |= (SAADC_CH_CONFIG_RESN_Pulldown << SAADC_CH_CONFIG_RESN_Pos);
					break;
				case ADC_PIN_CONN_VDD:
					chconfig |= (SAADC_CH_CONFIG_RESN_VDD1_2 << SAADC_CH_CONFIG_RESN_Pos);
					break;
			}
		}
		else
		{
			// Single ended
			resdiv = 1 << vResolution;
			NRF_SAADC->CH[pChanCfg[i].Chan].PSELN = 0;
		}

		//
		// *** Factor calculation
		// Vin = ADCresult * Reference / (Resolution * Gain)
		// => GainFactor = Reference / (Resolution * Gain)
		//

		// Reference voltage
		s_AdcnRF52DevData.RefVoltIdx[pChanCfg[i].Chan] = pChanCfg[i].RefVoltIdx;

		if (vpRefVolt[pChanCfg[i].RefVoltIdx].Type == ADC_REFVOLT_TYPE_INTERNAL)
		{
			chconfig |= SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos;
			s_AdcnRF52DevData.GainFactor[pChanCfg[i].Chan] = 0.6;
		}
		else
		{
			chconfig |= SAADC_CH_CONFIG_REFSEL_VDD1_4 << SAADC_CH_CONFIG_REFSEL_Pos;
			s_AdcnRF52DevData.GainFactor[pChanCfg[i].Chan] = vpRefVolt[s_AdcnRF52DevData.RefVoltIdx[pChanCfg[i].Chan]].Voltage;
		}

		if (pChanCfg[i].Gain & 0xFF)
		{
			// Factional gain
			s_AdcnRF52DevData.GainFactor[pChanCfg[i].Chan] /= (float)resdiv / (float)(pChanCfg[i].Gain & 0xFF);
			chconfig |= ((6 - (pChanCfg[i].Gain & 0xFF)) << SAADC_CH_CONFIG_GAIN_Pos) & SAADC_CH_CONFIG_GAIN_Msk;
		}
		else
		{
			s_AdcnRF52DevData.GainFactor[pChanCfg[i].Chan] /= (float)resdiv * (float)((pChanCfg[i].Gain >> 8)& 0xFF);
			chconfig |= ((5 + (31 - __builtin_clzl(pChanCfg[i].Gain >> 8))) << SAADC_CH_CONFIG_GAIN_Pos) & SAADC_CH_CONFIG_GAIN_Msk;
		}

		if (pChanCfg[i].PinP.Conn == ADC_PIN_CONN_VDD)
		{
			s_AdcnRF52DevData.GainFactor[pChanCfg[i].Chan] *= 2.0;
		}

		if (pChanCfg[i].AcqTime < 5)
		{
			// Acquisition time 3us not need to set.
		}
		else if (pChanCfg[i].AcqTime < 10)
		{
			// 5 us
			chconfig |= SAADC_CH_CONFIG_TACQ_5us << SAADC_CH_CONFIG_TACQ_Pos;
		}
		else if (pChanCfg[i].AcqTime < 15)
		{
			// 10 us
			chconfig |= SAADC_CH_CONFIG_TACQ_10us << SAADC_CH_CONFIG_TACQ_Pos;
		}
		else if (pChanCfg[i].AcqTime < 20)
		{
			// 15 us
			chconfig |= SAADC_CH_CONFIG_TACQ_15us << SAADC_CH_CONFIG_TACQ_Pos;
		}
		else if (pChanCfg[i].AcqTime < 40)
		{
			// 15 us
			chconfig |= SAADC_CH_CONFIG_TACQ_20us << SAADC_CH_CONFIG_TACQ_Pos;
		}
		else
		{
			chconfig |= SAADC_CH_CONFIG_TACQ_40us << SAADC_CH_CONFIG_TACQ_Pos;
		}

		if (pChanCfg[i].BurstMode == true)
		{
			chconfig |= SAADC_CH_CONFIG_BURST_Enabled << SAADC_CH_CONFIG_BURST_Pos;
		}

		NRF_SAADC->CH[pChanCfg[i].Chan].CONFIG = chconfig;

	}

	s_AdcnRF52DevData.NbChanAct = NbChan;

	NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos);

	return true;
}

/**
 * @brief	Close ADC channel
 * @param 	Chan : Channel number
 */
void AdcnRF52::CloseChannel(int Chan)
{
	NRF_SAADC->CH[Chan].PSELP = 0;
	NRF_SAADC->CH[Chan].PSELN = 0;
	NRF_SAADC->CH[Chan].CONFIG = 0;

	s_AdcnRF52DevData.ChanState[Chan] = 0;
}


bool AdcnRF52::StartConversion()
{
	if (nRF52ADCWaitBusy(10000) == false)
		return false;

	NRF_SAADC->RESULT.PTR = (uint32_t)s_AdcnRF52DevData.ResData;
	NRF_SAADC->RESULT.MAXCNT = s_AdcnRF52DevData.NbChanAct;
	NRF_SAADC->TASKS_START = 1;

	if (vbInterrupt == false)
	{
	    int timeout = 10000;
	    while (NRF_SAADC->EVENTS_STARTED == 0 && timeout-- > 0);
        NRF_SAADC->EVENTS_STARTED = 0;
	    NRF_SAADC->TASKS_SAMPLE = 1;
		s_AdcnRF52DevData.SampleCnt++;
	}

	return true;
}

void AdcnRF52::StopConversion()
{
	NRF_SAADC->TASKS_STOP = 1;

	nRF52ADCWaitForStop(10000);

	NRF_SAADC->RESULT.MAXCNT = 0;
}

int AdcnRF52::Read(ADC_DATA *pBuff, int Len)
{
	int cnt = 0;

	if (vbInterrupt)
	{
		for (int i = 0; i < SAADC_NRF52_MAX_CHAN && Len > 0; i++)
		{
			if (s_AdcnRF52DevData.ChanState[i] != 0)
			{
				if (s_AdcnRF52DevData.hFifo[i])
				{
					ADC_DATA *p = (ADC_DATA *)CFifoGet(s_AdcnRF52DevData.hFifo[i]);
					if (p != NULL)
					{
						memcpy(pBuff, p, sizeof(ADC_DATA));
						pBuff++;
						cnt ++;
						Len--;
					}
				}
				else
				{
					pBuff->Chan = i;
					//
					// *** Factor calculation
					// Vin = ADCresult * Reference / (Resolution * Gain)
					// => GainFactor = Reference / (Resolution * Gain)
					// => Vin = ADCresult * GainFactor
					pBuff->Data = (float)s_AdcnRF52DevData.ResData[i] * s_AdcnRF52DevData.GainFactor[i];
					pBuff->Timestamp = s_AdcnRF52DevData.SampleCnt;
					pBuff++;
					cnt++;
					Len--;
				}
			}
		}
	}
	else
	{
		if (nRF52ADCWaitForEnd(100000))
		{
			int timeout = 10000;
			while (NRF_SAADC->RESULT.AMOUNT <  NRF_SAADC->RESULT.MAXCNT && timeout-- > 0);

//			s_AdcnRF52DevData.SampleCnt++;

			for (int i = 0; i < SAADC_NRF52_MAX_CHAN && cnt < NRF_SAADC->RESULT.AMOUNT && cnt < Len; i++)
			{
				if (s_AdcnRF52DevData.ChanState[i] != 0)
				{

					pBuff->Chan = i;
					//
					// *** Factor calculation
					// Vin = ADCresult * Reference / (Resolution * Gain)
					// => GainFactor = Reference / (Resolution * Gain)
					// => Vin = ADCresult * GainFactor
					pBuff->Data = (float)s_AdcnRF52DevData.ResData[i] * s_AdcnRF52DevData.GainFactor[i];
					pBuff->Timestamp = s_AdcnRF52DevData.SampleCnt;
					pBuff++;
					cnt++;
				}
			}
			NRF_SAADC->RESULT.AMOUNT = 0;
			NRF_SAADC->EVENTS_DONE = 0;
			NRF_SAADC->EVENTS_RESULTDONE = 0;
		}
	}
	return cnt;
}

/**
 * @brief	Read ADC data of one channel
 *
 * @param 	Chan : Channel number
 *  		pBuff : Pointer to buffer for returning data
 *
 * @return	true - data available
 */
bool AdcnRF52::Read(int Chan, ADC_DATA *pBuff)
{
	if (pBuff == NULL || Chan < 0 || Chan >= SAADC_NRF52_MAX_CHAN)
		return false;

	if (vbInterrupt)
	{
		ADC_DATA *p = (ADC_DATA *)CFifoGet(s_AdcnRF52DevData.hFifo[Chan]);
		if (p == NULL)
		{
			pBuff->Chan = -1;
			return false;
		}
		memcpy(pBuff, p, sizeof(ADC_DATA));
	}
	else if (nRF52ADCWaitForEnd(100000))
	{
		int timeout = 100000;
		while (NRF_SAADC->RESULT.AMOUNT <  NRF_SAADC->RESULT.MAXCNT && timeout-- > 0);

//		s_AdcnRF52DevData.SampleCnt++;

		pBuff->Chan = Chan;
		//
		// *** Factor calculation
		// Vin = ADCresult * Reference / (Resolution * Gain)
		// => GainFactor = Reference / (Resolution * Gain)
		// => Vin = ADCresult * GainFactor
		pBuff->Data = (float)s_AdcnRF52DevData.ResData[Chan] * s_AdcnRF52DevData.GainFactor[Chan];
		pBuff->Timestamp = s_AdcnRF52DevData.SampleCnt;

		NRF_SAADC->RESULT.AMOUNT = 0;
		NRF_SAADC->EVENTS_DONE = 0;
		NRF_SAADC->EVENTS_RESULTDONE = 0;
	}

	return true;
}

