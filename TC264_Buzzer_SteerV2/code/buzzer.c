#include "buzzer.h"

/* CPU1 is the sole writer, CPU0 is the sole consumer of these messages. */
static volatile uint8 ring_requested = 0;
static volatile uint32 zebra_sequence = 0;
static uint8 zebra_latched = 0;
static uint8 zebra_clear_frames = 0;

/* Only the CPU0 interrupt owns the waveform state and GPIO writes. */
typedef enum
{
    BUZZER_IDLE,
    BUZZER_ZEBRA_GAP,
    BUZZER_ZEBRA_ON,
    BUZZER_ZEBRA_TAIL
} BuzzerPhase;

static BuzzerPhase phase = BUZZER_IDLE;
static uint32 consumed_sequence = 0;
static uint16 phase_ticks = 0;
static uint16 ring_ticks = 0;
static uint8 beeps_left = 0;
static uint8 ring_on = 0;
static uint8 output_high = 0;

static void buzzer_output(uint8 high)
{
    if (output_high != high)
    {
        output_high = high;
        gpio_set_level(BUZZER_PIN, high ? GPIO_HIGH : GPIO_LOW);
    }
}

void buzzer_init(void)
{
    ring_requested = 0;
    zebra_sequence = 0;
    zebra_latched = 0;
    zebra_clear_frames = 0;
    phase = BUZZER_IDLE;
    consumed_sequence = 0;
    phase_ticks = 0;
    ring_ticks = 0;
    beeps_left = 0;
    ring_on = 0;
    output_high = 0;
    gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

void buzzer_update_ring(uint8 active)
{
    ring_requested = (active != 0);
}

void buzzer_update_zebra(uint8 running, uint8 valid, uint8 visible)
{
    if (!running)
    {
        zebra_latched = 0;
        zebra_clear_frames = 0;
        return;
    }
    if (!valid)
    {
        /* Skipping detection in a ring does not prove we left a zebra. */
        zebra_clear_frames = 0;
        return;
    }
    if (visible)
    {
        zebra_clear_frames = 0;
        if (!zebra_latched)
        {
            zebra_latched = 1;
            /* A monotonic event sequence avoids a cross-core clear/set race. */
            zebra_sequence++;
        }
    }
    else if (zebra_latched)
    {
        if (zebra_clear_frames < BUZZER_ZEBRA_CLEAR_FRAMES)
            zebra_clear_frames++;
        if (zebra_clear_frames >= BUZZER_ZEBRA_CLEAR_FRAMES)
        {
            zebra_latched = 0;
            zebra_clear_frames = 0;
        }
    }
}

void buzzer_tick_5ms(uint8 running)
{
    /* Finish every five-beep sequence, including after the final-lap stop. */
    if (phase != BUZZER_IDLE)
    {
        if (phase_ticks > 0)
            phase_ticks--;
        if (phase_ticks > 0)
            return;

        if (phase == BUZZER_ZEBRA_GAP)
        {
            buzzer_output(1);
            phase = BUZZER_ZEBRA_ON;
            phase_ticks = BUZZER_ZEBRA_ON_MS / BUZZER_TICK_MS;
            return;
        }
        if (phase == BUZZER_ZEBRA_ON)
        {
            buzzer_output(0);
            beeps_left--;
            phase = beeps_left ? BUZZER_ZEBRA_GAP : BUZZER_ZEBRA_TAIL;
            phase_ticks = BUZZER_ZEBRA_OFF_MS / BUZZER_TICK_MS;
            return;
        }
        phase = BUZZER_IDLE;
    }

    if (consumed_sequence != zebra_sequence)
    {
        consumed_sequence++;
        beeps_left = BUZZER_ZEBRA_BEEPS;
        phase = BUZZER_ZEBRA_GAP;
        phase_ticks = BUZZER_ZEBRA_OFF_MS / BUZZER_TICK_MS;
        ring_ticks = 0;
        ring_on = 0;
        /* Separate the first zebra beep from a ring beep already in progress. */
        buzzer_output(0);
        return;
    }

    if (!running || !ring_requested)
    {
        ring_ticks = 0;
        ring_on = 0;
        buzzer_output(0);
        return;
    }

    if (ring_ticks > 0)
        ring_ticks--;
    if (ring_ticks == 0)
    {
        ring_on = !ring_on;
        buzzer_output(ring_on);
        ring_ticks = (ring_on ? BUZZER_RING_ON_MS : BUZZER_RING_OFF_MS) /
                     BUZZER_TICK_MS;
    }
}
