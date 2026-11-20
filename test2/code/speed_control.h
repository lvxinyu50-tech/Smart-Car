#ifndef _SPEED_CONTROL_H
#define _SPEED_CONTROL_H

#include "zf_common_headfile.h"

void speed_control_init(void);
void speed_control_start(void);
void speed_control_stop(void);
void speed_control_update_10ms(void);

void speed_control_set_target(float target_mps);
float speed_control_get_target(void);
float speed_control_get_ramped_target(void);

int speed_control_get_left_output(void);
int speed_control_get_right_output(void);

#endif
