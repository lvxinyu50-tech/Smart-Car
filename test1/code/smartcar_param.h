#ifndef _SMARTCAR_PARAM_H
#define _SMARTCAR_PARAM_H

/* Low-speed chassis parameters. Hardware calibration stays in each driver. */
#define CAR_TARGET_SPEED_MPS       (0.20f)

/* Mode 2 straight speed: edit this value, then compile and flash again. */
#define STRAIGHT_TARGET_SPEED_MPS  (0.20f)
#define SPEED_TARGET_MAX_MPS       (0.60f)
#define SPEED_RAMP_MPS_PER_S       (0.20f)
#define SPEED_CONTROL_DT_S         (0.01f)

/* Existing PI controller, reused without adding another speed controller. */
#define SPEED_LEFT_KP              (1800.0f)
#define SPEED_LEFT_KI              (1200.0f)
#define SPEED_RIGHT_KP             (1800.0f)
#define SPEED_RIGHT_KI             (1200.0f)
#define SPEED_INTEGRAL_LIMIT       (2.0f)
#define SPEED_OUTPUT_LIMIT         (1800)

/* Camera polarity must be confirmed on the real track before ground running. */
#define VISION_WHITE_TRACK         (1)
#define VISION_BINARY_THRESHOLD    (120)

/* Three rows in the lower half of the actual MT9V03X_H image. */
#define VISION_ROW_COUNT           (3)
#define VISION_ROW_NEAR            (MT9V03X_H - 10)
#define VISION_ROW_MIDDLE          ((MT9V03X_H * 3) / 4)
#define VISION_ROW_FAR             ((MT9V03X_H / 2) + 5)

#define VISION_WEIGHT_NEAR         (3)
#define VISION_WEIGHT_MIDDLE       (2)
#define VISION_WEIGHT_FAR          (1)
#define VISION_MIN_VALID_ROWS      (2)
#define VISION_MIN_TRACK_WIDTH     (8)
#define VISION_MAX_TRACK_WIDTH     (MT9V03X_W - 4)

/* Positive error means the road is right; this car then lowers servo PWM. */
#define VISION_STEER_SIGN          (-1)  /* Duty direction for positive image error. */
#define VISION_STEER_KP            (2.0f)
#define VISION_STEER_MAX_DELTA     (140)
#define VISION_FRAME_TIMEOUT_MS    (300) /* From VSYNC, including processing, ms. */
#define VISION_LOST_FRAMES         (2)   /* Consecutive invalid complete frames to latch. */

#define DISPLAY_PERIOD_MS          (200U)

#endif
