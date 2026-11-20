#ifndef TEST1_VISION_TRACK_H
#define TEST1_VISION_TRACK_H

#include "zf_device_mt9v03x.h"
#include "smartcar_param.h"

#define VISION_IMAGE_CENTER_X (MT9V03X_W / 2)

/* Foreground diagnostics from one processed image. -1 means missing boundary. */
extern volatile int16 track_left[VISION_ROW_COUNT], track_right[VISION_ROW_COUNT];
extern volatile int16 track_center[VISION_ROW_COUNT], track_road_center, track_error;
extern volatile uint8 track_row_valid[VISION_ROW_COUNT], track_valid, track_valid_rows;
extern volatile int32 steer_output;  /* Proposed absolute duty, not actuator feedback. */
extern uint8 track_binary[VISION_ROW_COUNT][MT9V03X_W];

typedef struct
{
    uint32 capture_id;      /* Sequence allocated at the accepted VSYNC. */
    uint32 started_ms;      /* VSYNC time on the 10 ms chassis clock. */
    uint32 loss_epoch;      /* Advances on each run of VISION_LOST_FRAMES failures. */
    int32 steer_offset;     /* Signed duty offset, zero on invalid results. */
    uint8 has_frame;
    uint8 valid;
} vision_result_t;

uint8 vision_track_init(void);                    /* CPU0, before PIT starts. */
uint8 vision_track_begin_vsync(uint32 now_ms);    /* ISR: admit the SDK capture. */
void vision_track_on_dma_complete(uint8 enabled); /* ISR: after the SDK ack/disable. */
void vision_track_task(void);                     /* Foreground only. */
void vision_track_process_image(const uint8 image[MT9V03X_H][MT9V03X_W]);
vision_result_t vision_track_result(void);        /* Coherent published control input. */
uint8 vision_track_camera_ok(void);
uint32 vision_track_capture_id(void);
uint32 vision_track_get_frame_count(void);        /* Accepted full DMA frames. */
uint8 vision_track_is_ready(void);                /* Latest result is valid and fresh. */
#endif
