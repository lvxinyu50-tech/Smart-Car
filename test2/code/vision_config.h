#ifndef TEST2_VISION_CONFIG_H
#define TEST2_VISION_CONFIG_H

/* Grayscale and pixel geometry. Initial field settings, not ground distances.
 * Each entry: image y (near to far), weight, minimum/nominal/maximum lane width in px. */
#define VISION_SCAN_ROWS(X) \
    X(110, 3, 12, 72, 114) \
    X(100, 3, 12, 66, 106) \
    X(90,  3, 10, 58,  94) \
    X(80,  3,  9, 51,  84) \
    X(70,  2,  8, 44,  74) \
    X(60,  2,  7, 38,  65) \
    X(50,  2,  6, 32,  56)
#define VISION_WHITE_TRACK        1      /* 1: road brighter than threshold, 0: darker. */
#define VISION_BINARY_THRESHOLD   120    /* Inherited gray level, 0..255. */
#define VISION_PAGE_MS            4000U  /* Foreground time per MASK/GRAY page in IDLE and KEY4, ms. */
#define VISION_TARGET_X           (MT9V03X_W / 2) /* Optical reference, image column. */
#define VISION_MAX_SHIFT          34     /* Adjacent row / same-row motion tolerance, px. */
#define VISION_OVERLAP_GAP        8      /* Maximum gap between adjacent intervals, px. */
#define VISION_AMBIGUITY          8      /* Reject similarly scored candidate intervals. */
#define VISION_MIN_VALID_ROWS     4      /* Two real boundaries on each counted row. */
#define VISION_START_FRAMES       3      /* Consecutive reliable normal images after KEY4. */
#define VISION_CURVE_TREND        12     /* Near/far center difference, px; not curvature. */
#define VISION_CURVE_ERROR        16     /* Absolute lateral error for lower speed, px. */
#define VISION_STEER_SIGN         (-1)   /* Electrical offset sign for road to the right. */
#define VISION_STEER_KP           2.0f   /* Duty units / pixel, inherited. */
#define VISION_STEER_KD           0.0f   /* Duty units * seconds / pixel, uses deltaError / dt. */
#define VISION_PD_MAX_GAP_MS      120U   /* Reset derivative after this interval, ms. */
#define VISION_STEER_MAX_DELTA    140    /* Max absolute offset from mechanical center. */
#define VISION_STRAIGHT_MPS       0.15f  /* Initial m/s, verify starting torque and tracking. */
#define VISION_CURVE_MPS          0.12f  /* Initial m/s. */
#define VISION_CROSS_MPS          0.12f  /* Initial m/s. */
#define VISION_SPEED_CAP_MPS      0.20f  /* Absolute vision target ceiling, m/s. */
#define VISION_STEER_ONLY         0      /* Set to 1 for physical steering checks with PWM zero. */
#define VISION_FRAME_TIMEOUT_MS   300U   /* Age of last processed image from DMA completion. */
#define VISION_ACQUIRE_MS         1200U  /* No reliable starting track, ms. */
#define VISION_LOST_FRAMES        2      /* Consecutive complete unreliable images. */
#define VISION_LOSS_MS            100U   /* Inhibit immediately; sustained failure limit, ms. */
#define VISION_CROSS_STRAIGHT     1      /* Assumed competition rule: 0 stops at suspected cross. */
#define VISION_WIDE_PERCENT       160    /* Width vs this row's reliable/nominal width, %. */
#define VISION_WING_PX            10     /* Required extra pixels on each side of entry-exit line. */
#define VISION_EXIT_DELTA         24     /* Max deviation from projected entry direction, px. */
#define VISION_CROSS_CONFIRM      3      /* Consecutive geometrically supported new frames. */
#define VISION_RECOVER_FRAMES     3      /* Consecutive ordinary-road images to leave junction. */
#define VISION_PREDICT_MS         100U   /* Last supported geometry, maximum extrapolation, ms. */
#define VISION_CROSS_LIMIT_MS     900U   /* Absolute PASS + REACQUIRE limit from first PASS, ms. */
#endif
