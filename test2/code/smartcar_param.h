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

#include "vision_config.h"

#define DISPLAY_PERIOD_MS          (200U)

#endif
