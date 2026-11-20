#ifndef SMARTCAR_ENCODER_H
#define SMARTCAR_ENCODER_H

#include "zf_common_typedef.h"
#include "driver_config.h"

/* Direction-corrected readings from one completed 10 ms sample. */
typedef struct
{
    int32 delta_pulses;    /* Signed pulses across the whole sample period. */
    int64 total_pulses;    /* Signed wheel travel since the odometry origin, pulses. */
    float speed_mps;       /* Signed wheel speed, m/s. */
    float distance_m;      /* Signed displacement since reset, m. */
} encoder_wheel_state_t;

typedef struct
{
    encoder_wheel_state_t left;   /* TIM5 wheel. */
    encoder_wheel_state_t right;  /* TIM6 wheel from the same update. */
} encoder_snapshot_t;

void encoder_init(void);                     /* Configure both counters before enabling PIT. */
void encoder_update(void);                   /* Call once per CCU60_CH0 period. */
void encoder_reset_odometry(void);           /* Zero both origins now, preserve speed sampling. */
encoder_snapshot_t encoder_get_snapshot(void); /* Coherent wheel pair, callable on CPU0. */
int64 encoder_get_left_pulses(void);         /* Signed left odometry, pulses. */
int64 encoder_get_right_pulses(void);        /* Signed right odometry, pulses. */
float encoder_get_left_speed_mps(void);      /* Last left sample, m/s. */
float encoder_get_right_speed_mps(void);     /* Last right sample, m/s. */

/* Compatibility with original declarations. The 32-bit totals saturate on read. */
void update_speed_and_distance(void);  /* Original PIT entry. */
void clear_distance(void);             /* Reset both odometry origins. */
int get_left_encoder(void);            /* Last left sample, signed pulses. */
int get_right_encoder(void);           /* Last right sample, signed pulses. */
int32 get_left_pulse_sum(void);        /* Left total, pulses, int32 range. */
int32 get_right_pulse_sum(void);       /* Right total, pulses, int32 range. */
float get_left_speed(void);            /* Left m/s. */
float get_right_speed(void);           /* Right m/s. */
float get_speed(void);                 /* Arithmetic mean of wheel speeds, m/s. */
float get_left_distance(void);         /* Left displacement, m. */
float get_right_distance(void);        /* Right displacement, m. */
float get_distance(void);              /* Arithmetic mean of wheel displacements, m. */

/* Historical single-wheel scales. External callers using both wheels should
 * migrate to the independent wheel readings before changing calibration. */
#define ENCODER_PULSE_PER_METER ENCODER_L_PULSES_PER_M                /* Left counts/m. */
#define ENCODER_SPEED_K    (1000.0f / ENCODER_SAMPLE_MS / ENCODER_L_PULSES_PER_M) /* (m/s)/count. */
#define ENCODER_DISTANCE_K (1.0f / ENCODER_L_PULSES_PER_M)           /* Left m/count. */

#endif
