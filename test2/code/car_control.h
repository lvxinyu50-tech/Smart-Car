#ifndef _CAR_CONTROL_H
#define _CAR_CONTROL_H

#include "zf_common_headfile.h"

typedef enum
{
    CAR_INIT = 0,
    CAR_IDLE,
    CAR_RUN,
    CAR_STOP,
    CAR_FAULT
} car_state_t;

typedef enum
{
    CAR_MODE_VISION = 1,
    CAR_MODE_STRAIGHT = 2
} car_mode_t;

/* Power-on value is zero. KEY2/KEY3 enable a mode; KEY1 always stops. */
extern volatile uint8 car_run_enable;

void car_control_init(void);
void car_control_enter_mode(car_mode_t mode);
void car_control_stop(void);
void car_control_fault(void);
void car_control_task(void);
void car_control_update_10ms(void);

uint8 car_is_running(void);
car_state_t car_control_get_state(void);
car_mode_t car_control_get_mode(void);
const char *car_control_get_state_text(void);
const char *car_control_get_mode_text(void);

#endif
