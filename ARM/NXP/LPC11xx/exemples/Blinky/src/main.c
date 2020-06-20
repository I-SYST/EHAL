#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "idelay.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"

#define SWCLK_TCK_PORT				0
#define SWCLK_TCK_PIN				14
#define SWCLK_TCK_PINOP				1

int main(void)
{

	IOPinConfig(SWCLK_TCK_PORT, SWCLK_TCK_PIN, SWCLK_TCK_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);

	while (1)
	{
		LPC_GPIO->SET[SWCLK_TCK_PORT] = (1 << SWCLK_TCK_PIN);
		nsDelay(300);
		LPC_GPIO->CLR[SWCLK_TCK_PORT] = (1 << SWCLK_TCK_PIN);
	}
/*
	IOPinConfig(0, BLUEIO_LED_BLUE_PIN, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinSet(0, BLUEIO_LED_BLUE_PIN);
	IOPinConfig(BLUEIO_LED_GREEN_PORT, 17, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinSet(BLUEIO_LED_GREEN_PORT, 17);//BLUEIO_LED_GREEN_PIN);
	IOPinConfig(0, BLUEIO_LED_RED_PIN, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinSet(0, BLUEIO_LED_RED_PIN);
  
	while(true)
	{
		IOPinClear(0, BLUEIO_LED_BLUE_PIN);
		usDelay(1000000);
		IOPinSet(0, BLUEIO_LED_BLUE_PIN);
		IOPinClear(BLUEIO_LED_GREEN_PORT, 17);//BLUEIO_LED_GREEN_PIN);
		usDelay(1000000);
		IOPinSet(BLUEIO_LED_GREEN_PORT, 17);//BLUEIO_LED_GREEN_PIN);
		IOPinClear(0, BLUEIO_LED_RED_PIN);
		usDelay(1000000);
		IOPinSet(0, BLUEIO_LED_RED_PIN);
		usDelay(1000000);
	}
	*/
}
