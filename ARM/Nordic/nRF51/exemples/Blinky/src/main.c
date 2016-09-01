#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "idelay.h"
#include "blueio_board.h"
#include "iopincfg.h"
#include "iopinctrl.h"

int main(void)
{

	IOPinConfig(0, BLUEIO_LED_BLUE, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinSet(0, BLUEIO_LED_BLUE);
	IOPinConfig(0, BLUEIO_LED_GREEN, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinSet(0, BLUEIO_LED_GREEN);
	IOPinConfig(0, BLUEIO_LED_RED, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinSet(0, BLUEIO_LED_RED);
  
	while(true)
	{
		IOPinClear(0, BLUEIO_LED_BLUE);
		usDelay(1000000);
		IOPinSet(0, BLUEIO_LED_BLUE);
		IOPinClear(0, BLUEIO_LED_GREEN);
		usDelay(1000000);
		IOPinSet(0, BLUEIO_LED_GREEN);
		IOPinClear(0, BLUEIO_LED_RED);
		usDelay(1000000);
		IOPinSet(0, BLUEIO_LED_RED);
		usDelay(1000000);
	}
}
