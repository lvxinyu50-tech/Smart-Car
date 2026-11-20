#ifndef SMARTCAR_LED_H
#define SMARTCAR_LED_H

#include "zf_driver_gpio.h"
#include "driver_config.h"

void led_init(void);                           /* Set LED1..4 to low/high/low/high. */
void led_set(gpio_pin_enum led, uint8 level);  /* Set GPIO level: zero low, nonzero high. */
void led_toggle(gpio_pin_enum led);            /* Invert the output level of this pin. */

#endif
