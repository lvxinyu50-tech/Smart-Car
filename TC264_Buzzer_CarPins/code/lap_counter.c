#include "lap_counter.h"

volatile unsigned char lap_count = 0;
volatile unsigned char banmaxian_on = 0;

static volatile unsigned char reset_requested = 0;
static unsigned short startup_frames = 0;
static unsigned char clear_frames = 0;
static unsigned char armed = 0;

void lap_counter_request_start(void)
{
    reset_requested = 1;
}

unsigned char lap_counter_start_pending(void)
{
    return reset_requested;
}

void lap_counter_update(unsigned char running,
                        unsigned char detection_valid,
                        unsigned char zebra_present)
{
    if (reset_requested)
    {
        startup_frames = 0;
        clear_frames = 0;
        armed = 0;
        lap_count = 0;
        banmaxian_on = 0;
        /* Publish acknowledgement after clearing the old finish flag. */
        reset_requested = 0;
        return;
    }

    if (!running || banmaxian_on)
        return;

    /* Preserve the original 300-image-frame launch exclusion. */
    if (startup_frames < LAP_START_IGNORE_FRAMES)
    {
        startup_frames++;
        clear_frames = 0;
        return;
    }

    /* A skipped detection in a roundabout is not proof of leaving a line. */
    if (!detection_valid)
    {
        clear_frames = 0;
        return;
    }

    if (zebra_present)
    {
        clear_frames = 0;
        if (armed)
        {
            armed = 0;
            lap_count++;
            if (lap_count >= LAP_TARGET_COUNT)
                banmaxian_on = 1;
        }
    }
    else if (!armed)
    {
        if (clear_frames < LAP_CLEAR_FRAMES)
            clear_frames++;
        if (clear_frames >= LAP_CLEAR_FRAMES)
            armed = 1;
    }
}
