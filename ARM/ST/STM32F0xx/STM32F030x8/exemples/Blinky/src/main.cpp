//============================================================================
// Name        : main.cpp
// Author      : Nguyen Hoan Hoang
// Version     :
// Copyright   : Copyright 2019, I-SYST
// Description : Hello World in C++
//============================================================================

#include "iopinctrl.h"
#include "idelay.h"

static const IOPINCFG s_Pins[] = {
	{0, 3, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{0, 4, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static int s_NbPins = sizeof(s_Pins) / sizeof(IOPINCFG);


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
	IOPinCfg(s_Pins, s_NbPins);

	while (1)
	{
		IOPinSet(0, 3);
		//for (int i = 0; i < 100000; i++);
		msDelay(1000);
		IOPinClear(0, 3);
		IOPinSet(0, 4);
		//for (int i = 0; i < 100000; i++);
		msDelay(1000);
		IOPinClear(0, 4);
		//msDelay(200);

	}

	return 0;
}
