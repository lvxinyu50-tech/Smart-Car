#include "encoder.h"
#include "zf_common_interrupt.h"
#include "zf_driver_encoder.h"
#include <stdint.h>

#if ENCODER_SAMPLE_MS <= 0
#error Encoder sample period must be positive
#endif

/* CPU0 only. Shared 64-bit totals, partial-interval offsets and paired
 * reads/writes are protected by saving/restoring the previous interrupt mask. */
typedef struct
{
    int32 delta;           /* Full-period signed pulses, for speed. */
    int32 reset_offset;    /* Pulse count already traveled at the most recent reset. */
    int64 total;           /* Signed pulses accumulated since the odometry origin. */
} wheel_counts_t;

static volatile wheel_counts_t left_counts;    /* TIM5 software state. */
static volatile wheel_counts_t right_counts;   /* TIM6 software state. */
static const float left_m_per_pulse = 1.0f / ENCODER_L_PULSES_PER_M;
static const float right_m_per_pulse = 1.0f / ENCODER_R_PULSES_PER_M;
static const float samples_per_second = 1000.0f / ENCODER_SAMPLE_MS;

/* Widen before applying the sign, including for an int16 input of -32768. */
static int32 read_pulses(encoder_index_enum timer, int reverse)
{
    int32 pulses = (int32)encoder_get_count(timer);
    return reverse ? -pulses : pulses;
}

/* Preserve the vendor read-then-clear sequence, once per wheel per PIT. */
static void sample_wheel(volatile wheel_counts_t *wheel, encoder_index_enum timer,
                         int reverse)
{
    int32 pulses = read_pulses(timer, reverse);
    encoder_clear_count(timer);
    wheel->delta = pulses;
    wheel->total += pulses - wheel->reset_offset;
    wheel->reset_offset = 0;
}

void encoder_init(void)
{
    const wheel_counts_t zero = {0, 0, 0};
    encoder_dir_init(ENCODER_L_TIMER, ENCODER_L_COUNT_PIN, ENCODER_L_DIR_PIN);
    encoder_dir_init(ENCODER_R_TIMER, ENCODER_R_COUNT_PIN, ENCODER_R_DIR_PIN);
    encoder_clear_count(ENCODER_L_TIMER);
    encoder_clear_count(ENCODER_R_TIMER);
    left_counts = zero;
    right_counts = zero;
}

void encoder_update(void)
{
    uint32 mask = interrupt_global_disable();
    sample_wheel(&left_counts, ENCODER_L_TIMER, ENCODER_L_REVERSE);
    sample_wheel(&right_counts, ENCODER_R_TIMER, ENCODER_R_REVERSE);
    interrupt_global_enable(mask);
}

void encoder_reset_odometry(void)
{
    uint32 mask = interrupt_global_disable();
    /* Exclude pre-reset travel on the next update. Keep the hardware counters
     * running until the PIT so the next speed still spans the entire period. */
    left_counts.reset_offset = read_pulses(ENCODER_L_TIMER, ENCODER_L_REVERSE);
    right_counts.reset_offset = read_pulses(ENCODER_R_TIMER, ENCODER_R_REVERSE);
    left_counts.total = 0;
    right_counts.total = 0;
    interrupt_global_enable(mask);
}

/* Convert a copied count pair outside the interrupt lock, using this wheel's scale. */
static void convert_wheel(encoder_wheel_state_t *state, float meters_per_pulse)
{
    state->distance_m = (float)state->total_pulses * meters_per_pulse;
    state->speed_mps = (float)state->delta_pulses * (meters_per_pulse * samples_per_second);
}

encoder_snapshot_t encoder_get_snapshot(void)
{
    encoder_snapshot_t state;     /* Both wheels from the same software update. */
    uint32 mask = interrupt_global_disable();
    state.left.delta_pulses = left_counts.delta;
    state.left.total_pulses = left_counts.total;
    state.right.delta_pulses = right_counts.delta;
    state.right.total_pulses = right_counts.total;
    interrupt_global_enable(mask);
    convert_wheel(&state.left, left_m_per_pulse);
    convert_wheel(&state.right, right_m_per_pulse);
    return state;
}

int64 encoder_get_left_pulses(void)
{
    uint32 mask = interrupt_global_disable();
    int64 pulses = left_counts.total;
    interrupt_global_enable(mask);
    return pulses;
}

int64 encoder_get_right_pulses(void)
{
    uint32 mask = interrupt_global_disable();
    int64 pulses = right_counts.total;
    interrupt_global_enable(mask);
    return pulses;
}

/* Each delta is one naturally aligned 32-bit load on TC264. */
float encoder_get_left_speed_mps(void)
{
    return (float)left_counts.delta * (left_m_per_pulse * samples_per_second);
}

float encoder_get_right_speed_mps(void)
{
    return (float)right_counts.delta * (right_m_per_pulse * samples_per_second);
}

/* Preserve the original return type without wrapping large signed totals. */
static int32 legacy_total(int64 total)
{
    if (total > INT32_MAX)
        return INT32_MAX;
    if (total < INT32_MIN)
        return INT32_MIN;
    return (int32)total;
}

void update_speed_and_distance(void) { encoder_update(); }
void clear_distance(void)            { encoder_reset_odometry(); }
int get_left_encoder(void)           { return left_counts.delta; }
int get_right_encoder(void)          { return right_counts.delta; }
int32 get_left_pulse_sum(void)       { return legacy_total(encoder_get_left_pulses()); }
int32 get_right_pulse_sum(void)      { return legacy_total(encoder_get_right_pulses()); }
float get_left_speed(void)           { return encoder_get_left_speed_mps(); }
float get_right_speed(void)          { return encoder_get_right_speed_mps(); }
float get_left_distance(void)        { return (float)encoder_get_left_pulses() * left_m_per_pulse; }
float get_right_distance(void)       { return (float)encoder_get_right_pulses() * right_m_per_pulse; }

float get_speed(void)
{
    encoder_snapshot_t sample = encoder_get_snapshot();
    return (sample.left.speed_mps + sample.right.speed_mps) * 0.5f;
}

float get_distance(void)
{
    encoder_snapshot_t sample = encoder_get_snapshot();
    return (sample.left.distance_m + sample.right.distance_m) * 0.5f;
}
