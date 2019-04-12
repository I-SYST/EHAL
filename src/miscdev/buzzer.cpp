/**-------------------------------------------------------------------------
@file	buzzer.cpp

@brief	Generic implementation of buzzer driver

@author	Hoang Nguyen Hoan
@date	May 22, 2018

@license

Copyright (c) 2018, I-SYST, all rights reserved

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
//#include <string.h>
#include "idelay.h"
#include "miscdev/buzzer.h"

// miliHz
static const uint32_t s_MidiNoteFreq[] = {
	    8176,     8662,     9177,     9723,    10301,    11562,    12250,    12978,    13750,    14568,
	   15434,    16352,    17324,    18354,    19445,    20602,    21827,    23125,    24500,    25957,
	   27500,    29135,    30868,    32703,    34648,    36708,    38891,    41203,    43654,    46249,
	   48999,    51913,    55000,    58270,    61735,    65406,    69296,    73416,    77782,    82407,
	   87307,    92499,    97999,   103826,   110000,   116541,   123471,   130813,   138591,   146832,
	  155563,   164814,   174614,   184997,   195998,   207652,   220000,   233082,   246942,   261626,
	  277183,   293665,   311127,   329628,   349228,   369994,   391995,   415305,   440000,   466164,
	  493883,   523251,   554365,   587330,   622254,   659255,   698456,   739989,   783991,   830609,
	  880000,   932328,   987767,  1046502,  1108730,  1174659,  1244508,  1318510,  1396913,  1479978,
	 1567982,  1661219,  1760000,  1864655,  1975533,  2093004,  2217461,  2349318,  2489016,  2637021,
	 2793826,  2959955,  3135963,  3322437,  3520000,  3729310,  3951066,  4186009,  4434922,  4698636,
	 4978032,  5274041,  5587652,  5919910,  6271927,  6644875,  7040000,  7458620,  7902133,  8372029,
	 8869844,  9397272,  9956063, 10548082, 11175304, 11839821, 12543854, 13289750, 14080000, 14917240,
	15804266, 16774035, 17739688, 18794545, 19912127, 21096164
};

static const int NbMidiNotes = sizeof(s_MidiNoteFreq) / sizeof(uint32_t);

bool Buzzer::Init(Pwm * const pPwm, int Chan)
{
	if (pPwm == nullptr)
	{
		return false;
	}

	vpPwm = pPwm;
	vChan = Chan;

	return true;
}

void Buzzer::Volume(int Volume)
{
	if (Volume >= 0 && Volume <= 100)
	{
		// max volume is at 50% duty cycle
		vDutyCycle = Volume >> 1;
		vpPwm->DutyCycle(vChan, vDutyCycle);
	}
}

/**
 * @brief	Play frequency
 *
 * @param	Freq :
 * @param	msDuration	: Play duration in msec.
 *							if != 0, wait for it then stop
 *							else let pwm running and return (no stop)
 */
void Buzzer::Play(uint32_t Freq, uint32_t msDuration)
{
	vpPwm->Frequency(Freq);
	vpPwm->DutyCycle(vChan, vDutyCycle);
	vpPwm->Start(msDuration);

	if (msDuration)
	{
		// Wait for duration completion
		usDelay(msDuration * 1000);
		//vpPwm->Stop();
	}
}

/**
 * @brief	Play Midi note
 *
 * @param	MidiNote : Midi note value 0-255
 * @param	msDuration	: Play duration in msec.
 *							if != 0, wait for it then stop
 *							else let running and return (no stop)
 */
void Buzzer::Play(uint8_t MidiNote, uint32_t msDuration)
{
	Play((s_MidiNoteFreq[MidiNote] + 500) / 1000, msDuration);
}

void Buzzer::Stop()
{
	vpPwm->Stop();
}



