#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "idelay.h"
#include "blueio_board.h"
#include "iopincfg.h"
#include "iopinctrl.h"
#include "pulse_train.h"

extern void initialise_monitor_handles(void);

#define GPIO_PIN_TEST

static const IOPINCFG s_ButPins[] = {
	{BLUEIO_BUT1_PORT, BLUEIO_BUT1_PIN, BLUEIO_BUT1_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// But 1
	{BLUEIO_BUT2_PORT, BLUEIO_BUT2_PIN, BLUEIO_BUT2_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// But 1
};

static const int s_NbBut = sizeof(s_ButPins) / sizeof(IOPINCFG);

PULSE_TRAIN_PIN g_IoPins[] = {
	{0, 2, 0}, {0, 3, 0}, {0, 4, 0}, {0, 5, 0}, {0, 6, 0}, {0, 7, 0}, {0, 8, 0}, {0, 11, 0},
	{0, 12, 0}, {0, 13, 0}, {0, 14, 0}, {0, 15, 0}, {0, 16, 0}, {0, 17, 0}, {0, 18, 0},
	{0, 19, 0}, {0, 20, 0}, {0, 22, 0}, {0, 23, 0}, {0, 24, 0}, {0, 25, 0}, {0, 26, 0},
	{0, 27, 0}, {0, 28, 0}, {0, 29, 0}, {0, 30, 0}, {0, 31, 0}
};

int g_NbIoPins = sizeof(g_IoPins) / sizeof(PULSE_TRAIN_PIN);

PULSE_TRAIN_CFG g_PulseTrainCfg = {
	g_IoPins,
	sizeof(g_IoPins) / sizeof(PULSE_TRAIN_PIN),
	100000,
	PULSE_TRAIN_POL_LOW
};

int main(void)
{
   // initialise_monitor_handles();
#ifdef GPIO_PIN_TEST
	PulseTrain(&g_PulseTrainCfg, 0);
#endif

	printf("hello world!\n");

	IOPinCfg(s_ButPins, s_NbBut);
	IOPinEnableInterrupt(0, 6, BLUEIO_BUT1_PORT, BLUEIO_BUT1_PIN, IOPINSENSE_LOW_TRANSITION, NULL);
	IOPinEnableInterrupt(1, 6, BLUEIO_BUT2_PORT, BLUEIO_BUT2_PIN, IOPINSENSE_LOW_TRANSITION, NULL);
	//IOPinSetSense(BLUEIO_BUT1_PORT, BLUEIO_BUT1_PIN, IOPINSENSE_LOW_TRANSITION);
	//IOPinSetSense(BLUEIO_BUT2_PORT, BLUEIO_BUT2_PIN, IOPINSENSE_LOW_TRANSITION);

	IOPinConfig(0, 30, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinClear(0, 30);
	IOPinConfig(0, BLUEIO_TAG_BME680_LED2_BLUE_PIN, 0,
			IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinSet(0, BLUEIO_TAG_BME680_LED2_BLUE_PIN);
	IOPinConfig(0, BLUEIO_TAG_BME680_LED2_GREEN_PIN, 0,
			IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinSet(0, BLUEIO_TAG_BME680_LED2_GREEN_PIN);
	IOPinConfig(0, BLUEIO_TAG_BME680_LED2_RED_PIN, 0,
			IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinClear(0, BLUEIO_TAG_BME680_LED2_RED_PIN);
  
	while(true)
	{
		int d = IOPinRead(0, BLUEIO_BUT1_PIN);
		//printf("%d\r\n", d);

		//__WFE();
		IOPinClear(0, BLUEIO_TAG_BME680_LED2_BLUE_PIN);
		usDelay(1000000);
		IOPinSet(0, BLUEIO_TAG_BME680_LED2_BLUE_PIN);
		IOPinClear(0, BLUEIO_TAG_BME680_LED2_GREEN_PIN);
		usDelay(1000000);
		IOPinSet(0, BLUEIO_TAG_BME680_LED2_GREEN_PIN);
		IOPinClear(0, BLUEIO_TAG_BME680_LED2_RED_PIN);
		usDelay(1000000);
		IOPinSet(0, BLUEIO_TAG_BME680_LED2_RED_PIN);
		usDelay(1000000);
	}
}

