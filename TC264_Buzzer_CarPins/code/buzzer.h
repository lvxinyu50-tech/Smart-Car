#ifndef CODE_BUZZER_H_
#define CODE_BUZZER_H_

#include "zf_driver_gpio.h"

/* Active buzzer, S8050 low-side switch: high = sound, low = silent. */
#define BUZZER_PIN                  (P11_11)
#define BUZZER_TICK_MS              5u
#define BUZZER_RING_ON_MS           150u
#define BUZZER_RING_OFF_MS          150u
#define BUZZER_ZEBRA_ON_MS          100u
#define BUZZER_ZEBRA_OFF_MS         100u
#define BUZZER_ZEBRA_BEEPS          5u
#define BUZZER_ZEBRA_CLEAR_FRAMES   10u

/* CPU0: initialize before enabling interrupts / releasing CPU1. */
void buzzer_init(void);
/* CPU1: publish recognition results; these functions never drive the pin. */
void buzzer_update_ring(uint8 active);
void buzzer_update_zebra(uint8 running, uint8 valid, uint8 visible);
/* CPU0: call exactly once per 5 ms interrupt. No delay or extra timer. */
void buzzer_tick_5ms(uint8 running);

#endif
