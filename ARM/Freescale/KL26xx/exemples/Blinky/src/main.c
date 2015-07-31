/*
 ============================================================================
 Name        : main.c
 Author      : N. H. Hoang
 Version     :
 Copyright   : I-SYST inc.
 Description : Hello World in C
 ============================================================================
 */

#include <stdio.h>

#include "istddef.h"
#include "fseccfg.h"

// Frescale Flash default security config data

__attribute__ ((section(".fseccfg"), used))
const FSECCFG g_FSecData = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

const VERS g_Version __attribute__ ((section(".Version"), used)) = {
	"Blinky", 0, BUILDNO, {0,}
};

/*
 *
 * Print a greeting message on standard output and exit.
 *
 * On embedded platforms this might require semi-hosting or similar.
 *
 * For example, for toolchains derived from GNU Tools for Embedded,
 * to enable semi-hosting, the following was added to the linker:
 *
 * --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lc -lm -lrdimon -Wl,--end-group
 *
 * Adjust it for other toolchains.
 *
 */

int
main(void)
{
  printf("Hello ARM World!" "\n");
  return 0;
}
