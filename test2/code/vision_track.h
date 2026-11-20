#ifndef TEST2_VISION_TRACK_H
#define TEST2_VISION_TRACK_H
#include "vision_road.h"
uint8 vision_track_init(void);
uint8 vision_track_begin_vsync(uint32 now_ms);
void vision_track_on_dma_complete(uint8 enabled);
void vision_track_task(void);
vision_result_t vision_track_result(void);
uint8 vision_track_camera_ok(void);
uint32 vision_track_capture_id(void);
uint32 vision_track_get_frame_count(void);
uint8 vision_track_is_ready(void);
uint32 vision_track_new_session(void); /* Called under CPU0 lock on mode transitions. */
#endif
