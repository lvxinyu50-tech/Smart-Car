#include "led.h"

void led_init(void)
{
    gpio_init(LED1, GPO, GPIO_LOW,  GPO_PUSH_PULL);
    gpio_init(LED2, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(LED3, GPO, GPIO_LOW,  GPO_PUSH_PULL);
    gpio_init(LED4, GPO, GPIO_HIGH, GPO_PUSH_PULL);
}

void led_set(gpio_pin_enum led, uint8 level)
{
    gpio_set_level(led, level);
}

void led_toggle(gpio_pin_enum led)
{
    gpio_toggle_level(led);
}
