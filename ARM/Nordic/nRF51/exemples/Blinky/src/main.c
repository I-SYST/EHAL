#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "idelay.h"
#include "blueio_board.h"
#include "iopincfg.h"
#include "iopinctrl.h"

int main(void)
{

	IOPinConfig(0, BLUEIO_LED_BLUE_PIN, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinSet(0, BLUEIO_LED_BLUE_PIN);
	IOPinConfig(0, BLUEIO_LED_GREEN_PIN, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinSet(0, BLUEIO_LED_GREEN_PIN);
	IOPinConfig(0, BLUEIO_LED_RED_PIN, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinSet(0, BLUEIO_LED_RED_PIN);
  
	while(true)
	{
		IOPinClear(0, BLUEIO_LED_BLUE_PIN);
		usDelay(1000000);
		IOPinSet(0, BLUEIO_LED_BLUE_PIN);
		IOPinClear(0, BLUEIO_LED_GREEN_PIN);
		usDelay(1000000);
		IOPinSet(0, BLUEIO_LED_GREEN_PIN);
		IOPinClear(0, BLUEIO_LED_RED_PIN);
		usDelay(1000000);
		IOPinSet(0, BLUEIO_LED_RED_PIN);
		usDelay(1000000);
	}
}
