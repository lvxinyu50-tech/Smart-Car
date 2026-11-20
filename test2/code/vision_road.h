#ifndef TEST2_VISION_ROAD_H
#define TEST2_VISION_ROAD_H
#include "zf_device_mt9v03x.h"
#include "vision_config.h"
#define COUNT_SCAN(y,w,lo,nom,hi) +1
enum { VISION_ROW_COUNT = 0 VISION_SCAN_ROWS(COUNT_SCAN) };
#undef COUNT_SCAN
#define VISION_MID (VISION_ROW_COUNT / 2)
#define VISION_LAST (VISION_ROW_COUNT - 1)

typedef enum { VISION_NORMAL, VISION_CROSS_DETECT, VISION_CROSS_PASS, VISION_REACQUIRE, VISION_LOST } vision_state_t;
typedef struct {
    int16 left, right, center, width; /* Pixel limits of one run. Missing geometry = -1. */
    int16 path_x; /* Center sample used by control, measured or supplied by use_path(). */
    uint8 left_seen, right_seen, touches_edge, nearly_white, candidates, associated, ambiguous, valid;
} vision_row_t;
typedef struct {
    uint32 capture_id, completed_ms, processed_ms; /* CPU0 10 ms clock, modulo 2^32. */
    uint32 completed_ticks; /* STM0 low word at final DMA; unsigned delta for PD. */
    uint32 generation, loss_epoch, bad_count, cross_fault_epoch, cross_entered_ms;
    int32 steer_offset;              /* Proposed signed duty units, relative to center. */
    float target_mps;                /* Target for both existing PI loops. */
    int16 center, error, trend;      /* Pixels; use valid before interpreting error. */
    uint8 has_frame, valid, valid_rows, weight, confidence, normal_streak, crossing;
    vision_state_t state;
} vision_result_t;

/* Only CPU0 foreground reads/writes these diagnostics; display never reads DMA memory. */
extern vision_row_t vision_rows[VISION_ROW_COUNT];
int vision_row_y(unsigned index);
uint8 vision_config_valid(void);
void vision_road_reset(void);
/* A requested display frame receives the full mask. Ordinary frames binarize scan rows only. */
void vision_road_process(const uint8 image[MT9V03X_H][MT9V03X_W], uint32 completed_ms, float tick_seconds,
                         vision_result_t *result, uint8 mask[][MT9V03X_W]);
#endif
