/**-------------------------------------------------------------------------
@file	temp_nrf5x.cpp

@brief	nRF5x on silicon temperature sensor implementation.

@author	Hoang Nguyen Hoan
@date	Apr. 5, 2019

@license

Copyright (c) 2019, I-SYST inc., all rights reserved

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
#include "nrf.h"

#include "temp_nrf5x.h"

static TempnRF5x *s_pTempnRF5x = NULL;

extern "C" void TEMP_IRQHandler(void)
{
	if (s_pTempnRF5x != NULL)
	{
		s_pTempnRF5x->IntHandler();
	}
    NVIC_ClearPendingIRQ(TEMP_IRQn);
}

void TempnRF5x::IntHandler()
{
	if (UpdateData() == true)
	{
		if (vDataRdyHandler != NULL)
		{
			vDataRdyHandler(this, &vData);
		}
	}
}

bool TempnRF5x::Init(const TEMPSENSOR_CFG &CfgData, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	vpTimer = pTimer;
	vpIntrf = NULL;
	vDataRdyHandler = CfgData.DataRdyCB;

	s_pTempnRF5x = this;

    NRF_TEMP->INTENSET = 1;

	NVIC_ClearPendingIRQ(TEMP_IRQn);
	NVIC_SetPriority(TEMP_IRQn, CfgData.IntPrio);
	NVIC_EnableIRQ(TEMP_IRQn);

	return true;
}

/**
 * @brief	Power on or wake up device
 *
 * @return	true - If success
 */
bool TempnRF5x::Enable()
{
    NRF_TEMP->INTENSET = 1;

	NVIC_ClearPendingIRQ(TEMP_IRQn);
	NVIC_EnableIRQ(TEMP_IRQn);

	return true;
}

/**
 * @brief	Put device in power down or power saving sleep mode
 */
void TempnRF5x::Disable()
{
    NRF_TEMP->INTENSET = 0;
	NVIC_DisableIRQ(TEMP_IRQn);
}

/**
 * @brief	Start sampling data
 *
 * This is a require implementation by sensor implementer.\n
 * This function initiates sensor to do actual measurement.
 *
 * @return	true - success
 * 			false - in case of error or sensor busy measuring
 */
bool TempnRF5x::StartSampling()
{
	NRF_TEMP->TASKS_START = 1;

	return true;
}

bool TempnRF5x::UpdateData()
{
	if (NRF_TEMP->EVENTS_DATARDY)
	{
		if (vpTimer != NULL)
		{
			vData.Timestamp = vpTimer->uSecond();
		}
		else
		{
			vData.Timestamp = 0;
		}
		vData.Temperature = (NRF_TEMP->TEMP * 25);

		NRF_TEMP->EVENTS_DATARDY = 0;

		return true;
	}

	return false;
}


