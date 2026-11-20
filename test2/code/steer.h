#ifndef SMARTCAR_STEER_H
#define SMARTCAR_STEER_H

#include "zf_common_typedef.h"
#include "driver_config.h"

/* Absolute PWM duty units, 0..PWM_DUTY_MAX (10000). */
void steer_init(void);                  /* Start PWM at the configured center. */
void steer_set_duty(uint32 duty);       /* Clamp absolute duty to both mechanical limits. */
void steer_set_offset(int32 offset);    /* Signed duty units from center, positive toward left. */
void steer_center(void);                /* Command STEER_MIDDLE. */
uint32 steer_get_center_duty(void);     /* Return configured center, duty units. */
void steer_duty(uint32 duty);           /* Original absolute duty entry point. */

#endif
