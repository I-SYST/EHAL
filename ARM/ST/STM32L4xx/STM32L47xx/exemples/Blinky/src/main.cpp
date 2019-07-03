//============================================================================
// Name        : main.cpp
// Author      : Nguyen Hoan Hoang
// Version     :
// Copyright   : Copyright 2019, I-SYST inc.
// Description : Hello World in C++
//============================================================================
#include "coredev/iopincfg.h"
#include "iopinctrl.h"

#define BUT1_PORT	3
#define BUT1_PIN	0
#define BUT1_PINOP	0
#define BUT2_PORT	3
#define BUT2_PIN	1
#define BUT2_PINOP	0
#define BUT3_PORT	3
#define BUT3_PIN	2
#define BUT3_PINOP	0
#define BUT4_PORT	3
#define BUT4_PIN	3
#define BUT4_PINOP	0

static const IOPINCFG s_ButPins[] = {
	{BUT1_PORT, BUT1_PIN, BUT1_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// But 1
	{BUT2_PORT, BUT2_PIN, BUT2_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// But 2
	{BUT3_PORT, BUT3_PIN, BUT3_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// But 2
	{BUT4_PORT, BUT4_PIN, BUT4_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// But 2
};

static const int s_NbBut = sizeof(s_ButPins) / sizeof(IOPINCFG);

void ButHandler(int IntNo)
{
	printf("But Int\r\n");
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
	IOPinCfg(s_ButPins, s_NbBut);
	IOPinEnableInterrupt(1, 3, BUT2_PORT, BUT2_PIN, IOPINSENSE_LOW_TRANSITION, ButHandler);

	while (1)
	{
		__WFE();
		if (IOPinRead(BUT1_PORT, BUT1_PIN) == 0)
		{
			printf("but1 presssed\r\n");
		}
		if (IOPinRead(BUT2_PORT, BUT2_PIN) == 0)
		{
			printf("but2 presssed\r\n");
		}
		if (IOPinRead(BUT3_PORT, BUT3_PIN) == 0)
		{
			printf("but3 presssed\r\n");
		}
		if (IOPinRead(BUT4_PORT, BUT4_PIN) == 0)
		{
			printf("but4 presssed\r\n");
		}
	}

	return 0;
}
