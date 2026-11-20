#ifndef TEST2_VISION_DEBUG_H
#define TEST2_VISION_DEBUG_H

#include "vision_road.h"

/* CPU0 foreground owns this storage. DMA and the control ISR never write it. */
typedef struct {
    uint8 gray[MT9V03X_H][MT9V03X_W];
    uint8 binary[MT9V03X_H][MT9V03X_W]; /* 255 = road, 0 = background. */
    vision_row_t rows[VISION_ROW_COUNT];
    vision_result_t result;
} vision_debug_frame_t;

/* Request only after finishing the previous screen transfer. Repeated requests coalesce.
 * The returned frame stays immutable until a subsequent request is serviced. */
void vision_debug_request(void);
const vision_debug_frame_t *vision_debug_latest(void); /* NULL before first frame or after session change. */
#endif
