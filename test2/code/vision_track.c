/* MT9V03X V3.4.5: VSYNC arms DMA5; only the final SDK callback disables
 * DMA and asserts finish. READY owns the single buffer until analysis ends. */
#include "vision_track.h"
#include "vision_debug.h"
#include "test_mode/test_mode.h"
#include "zf_common_interrupt.h"
#include "IfxStm.h"
#include <string.h>

enum { FRAME_FREE, FRAME_CAPTURING, FRAME_READY };
static volatile uint8 frame_state, camera_ok, crossed_vsync;
static volatile uint32 capture_id, completed_ms, completed_count, generation, capture_generation;
static volatile vision_result_t published;
static vision_debug_frame_t debug_frame;
static uint8 debug_requested; /* Foreground only. */
static volatile uint32 completed_ticks;
static float tick_seconds;
static uint32 history_generation;
static uint8 history_ready;

uint8 vision_track_init(void)
{
    vision_road_reset();
    if (!vision_config_valid()) return 0;
    mt9v03x_finish_flag = 0;
    tick_seconds = 1.0f / IfxStm_getFrequency(&MODULE_STM0);
    camera_ok = mt9v03x_init() == 0;
    return camera_ok;
}
uint32 vision_track_new_session(void) { return ++generation; }
uint8 vision_track_begin_vsync(uint32 now_ms)
{
    (void)now_ms;
    if (!camera_ok || frame_state == FRAME_READY) return 0;
    if (frame_state == FRAME_CAPTURING) { crossed_vsync = 1; return 0; }
    crossed_vsync = 0;
    capture_generation = generation;
    ++capture_id;
    frame_state = FRAME_CAPTURING;
    return 1;
}
void vision_track_on_dma_complete(uint8 enabled)
{
    if (frame_state != FRAME_CAPTURING || enabled) return;
    if (mt9v03x_finish_flag && !crossed_vsync)
    {
        completed_ticks = IfxStm_getLower(&MODULE_STM0);
        completed_ms = test_mode_time_ms();
        ++completed_count;
        frame_state = FRAME_READY;
    }
    else { mt9v03x_finish_flag = 0; frame_state = FRAME_FREE; }
}
vision_result_t vision_track_result(void)
{
    uint32 mask = interrupt_global_disable();
    vision_result_t r = published;
    interrupt_global_enable(mask);
    return r;
}
void vision_debug_request(void) { debug_requested = 1; }
const vision_debug_frame_t *vision_debug_latest(void)
{
    return debug_frame.result.has_frame && debug_frame.result.generation == generation ? &debug_frame : NULL;
}
uint8 vision_track_camera_ok(void) { return camera_ok; }
uint32 vision_track_capture_id(void) { return capture_id; }
uint32 vision_track_get_frame_count(void) { return completed_count; }
uint8 vision_track_is_ready(void)
{
    vision_result_t r = vision_track_result();
    return camera_ok && r.has_frame && r.valid && r.generation == generation &&
           test_mode_time_ms() - r.completed_ms < VISION_FRAME_TIMEOUT_MS;
}
/* Caller holds the short CPU0 lock. The SDK has finished this transfer. */
static void release_frame(void)
{
    mt9v03x_finish_flag = 0;
    frame_state = FRAME_FREE;
}

void vision_track_task(void)
{
    vision_result_t r = {0};
    uint8 copied = 0;
    uint32 mask;
    if (frame_state != FRAME_READY) return;
    r.generation = capture_generation;
    if (r.generation == generation)
    {
        if (!history_ready || r.generation != history_generation)
        {
            vision_road_reset();
            history_generation = r.generation;
            history_ready = 1;
        }
        r.capture_id = capture_id;
        r.has_frame = 1;
        r.completed_ms = completed_ms;
        r.completed_ticks = completed_ticks;
        if (debug_requested)
        {
            /* READY blocks new DMA starts during this copy. Interrupts remain enabled. */
            memcpy(debug_frame.gray, mt9v03x_image, sizeof(debug_frame.gray));
            mask = interrupt_global_disable();
            release_frame();
            interrupt_global_enable(mask);
            copied = 1;
            debug_requested = 0;
            vision_road_process(debug_frame.gray, r.completed_ms, tick_seconds, &r, debug_frame.binary);
            memcpy(debug_frame.rows, vision_rows, sizeof(debug_frame.rows));
        }
        else
            vision_road_process(mt9v03x_image, r.completed_ms, tick_seconds, &r, NULL);
        r.processed_ms = test_mode_time_ms();
        if (copied) debug_frame.result = r;
    }
    mask = interrupt_global_disable();
    if (r.has_frame && r.generation == generation) published = r;
    if (!copied) release_frame(); /* A later DMA may already be CAPTURING or READY. */
    interrupt_global_enable(mask);
}
