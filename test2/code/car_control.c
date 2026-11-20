/*
 * Two-mode vehicle control.
 * KEY2 release: mode 1, camera tracking.
 * KEY3 release: mode 2, straight driving with steering at the middle value.
 * KEY1 release: immediate stop in either mode.
 * Mode changes reset the old speed controller before new output is allowed.
 */

#include "car_control.h"
#include "speed_control.h"
#include "vision_track.h"
#include "smartcar_param.h"
#include "steer.h"
#include "motor.h"
#include "key.h"

static volatile car_state_t car_state = CAR_INIT;
static volatile car_mode_t car_mode = CAR_MODE_VISION;
static volatile uint8 pit_run_active = 0;
static volatile uint8 control_reset_pending = 1;

volatile uint8 car_run_enable = 0;

void car_control_init(void)
{
    car_state = CAR_IDLE;
    car_mode = CAR_MODE_VISION;
    car_run_enable = 0;
    pit_run_active = 0;
    control_reset_pending = 1;

    speed_control_init();
    speed_control_set_target(CAR_TARGET_SPEED_MPS);
    steer_duty(STEER_MIDDLE);
    motor_duty(0, 0);
}

void car_control_enter_mode(car_mode_t mode)
{
    if(CAR_FAULT == car_state)
        return;

    /* Stop old output first. The PIT completes the controller reset. */
    car_run_enable = 0;
    car_state = CAR_STOP;
    control_reset_pending = 1;
    motor_duty(0, 0);
    steer_duty(STEER_MIDDLE);

    car_mode = mode;
    if(CAR_MODE_STRAIGHT == mode)
        speed_control_set_target(STRAIGHT_TARGET_SPEED_MPS);
    else
        speed_control_set_target(CAR_TARGET_SPEED_MPS);

    car_state = CAR_RUN;
    car_run_enable = 1;
}

void car_control_stop(void)
{
    car_run_enable = 0;
    car_state = CAR_STOP;
    control_reset_pending = 1;
    steer_duty(STEER_MIDDLE);
    motor_duty(0, 0);
}

void car_control_fault(void)
{
    car_run_enable = 0;
    car_state = CAR_FAULT;
    control_reset_pending = 1;
    steer_duty(STEER_MIDDLE);
    motor_duty(0, 0);
}

void car_control_task(void)
{
    int key_event = get_key_value();

    if(1 == key_event)
    {
        car_control_stop();
    }
    else if(2 == key_event)
    {
        car_control_enter_mode(CAR_MODE_VISION);
    }
    else if(3 == key_event)
    {
        car_control_enter_mode(CAR_MODE_STRAIGHT);
    }
}

void car_control_update_10ms(void)
{
    uint8 mode_can_run;

    if(control_reset_pending)
    {
        speed_control_stop();
        pit_run_active = 0;
        control_reset_pending = 0;
    }

    mode_can_run = (CAR_MODE_STRAIGHT == car_mode) ?
                   1 : vision_track_is_ready();

    if((CAR_RUN == car_state) && car_run_enable && mode_can_run)
    {
        if(!pit_run_active)
        {
            speed_control_start();
            pit_run_active = 1;
        }

        speed_control_update_10ms();

        if(CAR_MODE_STRAIGHT == car_mode)
            steer_duty(STEER_MIDDLE);
    }
    else
    {
        if(pit_run_active)
        {
            speed_control_stop();
            pit_run_active = 0;

            /* Only vision loss latches STOP. Straight mode ignores vision. */
            if((CAR_RUN == car_state) && (CAR_MODE_VISION == car_mode))
            {
                car_run_enable = 0;
                car_state = CAR_STOP;
            }
        }
        else
        {
            motor_duty(0, 0);
        }

        steer_duty(STEER_MIDDLE);
    }
}

uint8 car_is_running(void)
{
    return ((CAR_RUN == car_state) && car_run_enable) ? 1 : 0;
}

car_state_t car_control_get_state(void)
{
    return car_state;
}

car_mode_t car_control_get_mode(void)
{
    return car_mode;
}

const char *car_control_get_state_text(void)
{
    switch(car_state)
    {
        case CAR_IDLE:
            return "IDLE ";
        case CAR_RUN:
            return "RUN  ";
        case CAR_STOP:
            return "STOP ";
        case CAR_FAULT:
            return "FAULT";
        default:
            return "INIT ";
    }
}

const char *car_control_get_mode_text(void)
{
    return (CAR_MODE_STRAIGHT == car_mode) ?
           "MODE2 STRAIGHT" : "MODE1 VISION";
}
