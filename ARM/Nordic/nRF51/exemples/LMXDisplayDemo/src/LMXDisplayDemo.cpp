//============================================================================
// Name        : LMXDisplayDemo.cpp
// Author      : Nguyen Hoan Hoang
// Version     :
// Copyright   : Copyright 2014, I-SYST inc
// Description : Hello World in C++
//============================================================================

#include <iostream>
#include "ledmxio.h"

using namespace std;

// I/O pins connection
LEDMXIOCFG g_IOCfg = {
  21,  // WR pin
  22,  // RD pin
  23,  // Data pin
  24, // En pin
  { 25, 26, 27, 29,}, // CS pins
  4,  // Number of CS pins
  LEDMX_CSTYPE_GPIO
};

// Display board configuration
LEDMXCFG g_Cfg = {
  &g_IOCfg,
  1,  // Number of display board in daisy chain, only one in this case
  {0, 1, 2, 3, 4, 5, 6, 7}, // display board ordering
};

/*LEDMXDEV g_LmxDev = {0,};

LEDMXCFG g_Cfg1 = {
  &g_IOCfg,
  4,  // Number of display board in daisy chain
  {4, 5, 6, 7, }, // display board ordering
};
LEDMXDEV g_LmxDev1 = {0,};*/

LedMx g_LmxDev;

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

int
main()
{

	g_LmxDev.Init(g_Cfg);

	g_LmxDev.PrintLeft("Hello World");

	return 0;
}
