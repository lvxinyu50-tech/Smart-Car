/*
 * MT9V03X basic vision tracking.
 * Acquisition stays in the Seekfree driver; this module consumes full frames.
 * The DMA ISR marks a completed frame and VSYNC is held until processing ends.
 * Therefore the driver never overwrites a frame that is being processed.
 */

#include "vision_track.h"
#include "steer.h"
#include "test_mode/test_mode.h"
#include "zf_common_interrupt.h"

volatile int16 track_left[VISION_ROW_COUNT];
volatile int16 track_right[VISION_ROW_COUNT];
volatile int16 track_center[VISION_ROW_COUNT];
volatile int16 track_road_center = -1;
volatile int16 track_error = 0;
volatile uint8 track_valid = 0;
volatile uint8 track_valid_rows = 0;
volatile uint8 track_row_valid[VISION_ROW_COUNT];
volatile int32 steer_output = STEER_MIDDLE;

uint8 track_binary[VISION_ROW_COUNT][MT9V03X_W];

/* Single DMA buffer: FREE -> CAPTURING -> READY -> FREE after processing. */
enum { FRAME_FREE, FRAME_CAPTURING, FRAME_READY };
static volatile uint8 frame_state, camera_ok, crossed_vsync;
static volatile uint32 capture_id, started_ms, completed_frames;
static volatile vision_result_t published;
static uint8 invalid_streak; /* Foreground only; saturates at the trip count. */
static int16 last_road_center = VISION_IMAGE_CENTER_X;

static const int16 vision_rows[VISION_ROW_COUNT] =
{
    VISION_ROW_NEAR,
    VISION_ROW_MIDDLE,
    VISION_ROW_FAR
};

static const uint8 vision_weights[VISION_ROW_COUNT] =
{
    VISION_WEIGHT_NEAR,
    VISION_WEIGHT_MIDDLE,
    VISION_WEIGHT_FAR
};

#if ((VISION_ROW_NEAR < 0) || (VISION_ROW_NEAR >= MT9V03X_H) || \
     (VISION_ROW_MIDDLE < 0) || (VISION_ROW_MIDDLE >= MT9V03X_H) || \
     (VISION_ROW_FAR < 0) || (VISION_ROW_FAR >= MT9V03X_H))
#error "Vision row is outside the real MT9V03X image."
#endif

#if ((VISION_MIN_VALID_ROWS < 1) || \
     (VISION_MIN_VALID_ROWS > VISION_ROW_COUNT))
#error "VISION_MIN_VALID_ROWS is invalid."
#endif

#if VISION_ROW_COUNT != 3 || VISION_WEIGHT_NEAR < 1 || VISION_WEIGHT_MIDDLE < 1 || VISION_WEIGHT_FAR < 1 || (VISION_WEIGHT_NEAR + VISION_WEIGHT_MIDDLE + VISION_WEIGHT_FAR) > 255
#error Invalid three-row vision weights
#endif
#if VISION_LOST_FRAMES < 1 || VISION_LOST_FRAMES > 255 || VISION_BINARY_THRESHOLD < 0 || VISION_BINARY_THRESHOLD > 255 || (VISION_WHITE_TRACK != 0 && VISION_WHITE_TRACK != 1)
#error Invalid vision loss, threshold or polarity setting
#endif
#if (VISION_STEER_SIGN != -1 && VISION_STEER_SIGN != 1) || VISION_STEER_MAX_DELTA < 0 || STEER_MIDDLE - VISION_STEER_MAX_DELTA < STEER_R_MIN || STEER_MIDDLE + VISION_STEER_MAX_DELTA > STEER_L_MAX
#error Invalid steering sign or range
#endif

uint8 vision_track_init(void)
{
    uint8 i;
    for (i = 0; i < VISION_ROW_COUNT; ++i)
        track_left[i] = track_right[i] = track_center[i] = -1;
    mt9v03x_finish_flag = 0;
    camera_ok = (mt9v03x_init() == 0);
    return camera_ok;
}

/* A second VSYNC before completion makes the current image suspect. Leave
 * DMA alone until the SDK finishes or disables it on request loss. */
uint8 vision_track_begin_vsync(uint32 now_ms)
{
    if (!camera_ok || frame_state == FRAME_READY)
        return 0;
    if (frame_state == FRAME_CAPTURING)
    {
        crossed_vsync = 1;
        return 0;
    }
    crossed_vsync = 0;
    started_ms = now_ms;
    ++capture_id;
    frame_state = FRAME_CAPTURING;
    return 1;
}

void vision_track_on_dma_complete(uint8 enabled)
{
    if (frame_state != FRAME_CAPTURING || enabled)
        return;  /* Intermediate linked-list segment or no active transfer. */
    if (mt9v03x_finish_flag && !crossed_vsync)
    {
        ++completed_frames;
        frame_state = FRAME_READY;
    }
    else
    {
        mt9v03x_finish_flag = 0;
        frame_state = FRAME_FREE;
    }
}

vision_result_t vision_track_result(void)
{
    uint32 mask = interrupt_global_disable();
    vision_result_t result = published;
    interrupt_global_enable(mask);
    return result;
}

uint8 vision_track_camera_ok(void) { return camera_ok; }
uint32 vision_track_capture_id(void) { return capture_id; }
uint32 vision_track_get_frame_count(void) { return completed_frames; }

uint8 vision_track_is_ready(void)
{
    vision_result_t r = vision_track_result();
    return camera_ok && r.has_frame && r.valid &&
           (uint32)(test_mode_time_ms() - r.started_ms) < VISION_FRAME_TIMEOUT_MS;
}

static void vision_binarize_row(
    const uint8 image[MT9V03X_H][MT9V03X_W], uint8 row_index)
{
    int16 x;
    int16 y = vision_rows[row_index];

    for(x = 0; x < MT9V03X_W; x++)
    {
#if VISION_WHITE_TRACK
        track_binary[row_index][x] =
            (image[y][x] > VISION_BINARY_THRESHOLD) ? 255 : 0;
#else
        track_binary[row_index][x] =
            (image[y][x] <= VISION_BINARY_THRESHOLD) ? 255 : 0;
#endif
    }
}

/* Follow the nearest bounded run of track pixels. Ignore edge-touching
 * background and narrow speckles so they cannot hide a usable lane. */
static uint8 vision_extract_edge(uint8 row_index, int16 search_center)
{
    const uint8 *row = track_binary[row_index];
    int16 x = 0, best_distance = MT9V03X_W;
    while (x < MT9V03X_W)
    {
        int16 left, right, distance, width;
        while (x < MT9V03X_W && !row[x]) ++x;
        left = x;
        while (x < MT9V03X_W && row[x]) ++x;
        right = x - 1;
        width = x - left;
        if (left == 0 || x == MT9V03X_W || width < VISION_MIN_TRACK_WIDTH ||
            width > VISION_MAX_TRACK_WIDTH) continue;
        distance = (search_center < left) ? left - search_center :
                   ((search_center > right) ? search_center - right : 0);
        if (distance < best_distance)
        {
            best_distance = distance;
            track_left[row_index] = left - 1;
            track_right[row_index] = right + 1;
            track_center[row_index] = (left + right) / 2;
        }
    }
    return best_distance < MT9V03X_W;
}

/*
 * 纯图像处理：逐行提取边线，只对有效行重新归一化加权。
 * 中线偏右时 error 为正；本车右转为 PWM 减小，所以 P 输出取负号。
 */
void vision_track_process_image(
    const uint8 image[MT9V03X_H][MT9V03X_W])
{
    uint8 i;
    uint8 valid_count = 0;
    uint8 weight_sum = 0;
    int32 weighted_center = 0;
    int16 search_center = last_road_center;
    float output;

    track_valid_rows = 0;

    for(i = 0; i < VISION_ROW_COUNT; i++)
    {
        track_left[i] = -1;
        track_right[i] = -1;
        track_center[i] = -1;

        vision_binarize_row(image, i);

        track_row_valid[i] = vision_extract_edge(i, search_center);
        if(track_row_valid[i])
        {
            valid_count++;
            weighted_center += track_center[i] * vision_weights[i];
            weight_sum += vision_weights[i];
            search_center = track_center[i];
        }
    }

    track_valid_rows = valid_count;

    if((valid_count < VISION_MIN_VALID_ROWS) || (0 == weight_sum))
    {
        track_valid = 0;
        track_road_center = -1;
        track_error = 0;
        steer_output = STEER_MIDDLE;
        last_road_center = VISION_IMAGE_CENTER_X;
        return;
    }

    track_road_center =
        (weighted_center + (weight_sum / 2)) / weight_sum;
    track_error = track_road_center - VISION_IMAGE_CENTER_X;

    output = (float)STEER_MIDDLE +
             (VISION_STEER_SIGN * VISION_STEER_KP * (float)track_error);

    if(output > (float)(STEER_MIDDLE + VISION_STEER_MAX_DELTA))
        output = STEER_MIDDLE + VISION_STEER_MAX_DELTA;
    else if(output < (float)(STEER_MIDDLE - VISION_STEER_MAX_DELTA))
        output = STEER_MIDDLE - VISION_STEER_MAX_DELTA;

    steer_output = (int32)output;
    last_road_center = track_road_center;
    track_valid = 1;
}

/* The READY state owns the SDK buffer through processing. Only the result
 * publication and buffer hand-back mask interrupts, never the scan loops. */
void vision_track_task(void)
{
    uint32 mask;
    if (frame_state != FRAME_READY)
        return;
    vision_track_process_image(mt9v03x_image);
    mask = interrupt_global_disable();
    published.capture_id = capture_id;
    published.started_ms = started_ms;
    published.valid = track_valid;
    published.has_frame = 1;
    published.steer_offset = steer_output - STEER_MIDDLE;
    if (track_valid)
        invalid_streak = 0;
    else if (invalid_streak < VISION_LOST_FRAMES && ++invalid_streak == VISION_LOST_FRAMES)
        ++published.loss_epoch;
    mt9v03x_finish_flag = 0;
    frame_state = FRAME_FREE;
    interrupt_global_enable(mask);
}
