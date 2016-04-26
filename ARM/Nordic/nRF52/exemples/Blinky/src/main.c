/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup blinky_example_main main.c
 * @{
 * @ingroup blinky_example
 * @brief Blinky Example Application main file.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"

#define LED_0	30
#define LED_1	29
#define LED_2	28

int main(void)
{

  nrf_gpio_cfg_output(LED_0);
  nrf_gpio_pin_set(LED_0);
  nrf_gpio_cfg_output(LED_1);
  nrf_gpio_pin_set(LED_1);
  nrf_gpio_cfg_output(LED_2);
  nrf_gpio_pin_set(LED_2);

  while(true)
  {
    nrf_gpio_pin_clear(LED_0);
    nrf_delay_us(1000000);
    nrf_gpio_pin_set(LED_0);
    nrf_gpio_pin_clear(LED_1);
    nrf_delay_us(1000000);
    nrf_gpio_pin_set(LED_1);
    nrf_gpio_pin_clear(LED_2);
    nrf_delay_us(1000000);
    nrf_gpio_pin_set(LED_2);
  }
}
