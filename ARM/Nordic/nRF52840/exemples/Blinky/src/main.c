#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "idelay.h"
#include "blueio_board.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"

extern void initialise_monitor_handles(void);

int main(void)
{
   // initialise_monitor_handles();

    printf("hello world!\n");

	IOPinConfig(0, 13, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinSet(0, 13);
	IOPinConfig(0, 14, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinSet(0, 14);
	IOPinConfig(0, 15, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinSet(0, 15);
  
	while(true)
	{
		IOPinClear(0, 13);
		usDelay(1000000);
		IOPinSet(0, 14);
		IOPinClear(0, 15);
		usDelay(1000000);
		IOPinSet(0, 13);
		IOPinClear(0, 14);
		usDelay(1000000);
		IOPinSet(0, 15);
		usDelay(1000000);
	}
}

