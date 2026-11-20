/* Board wiring and driver calibration for the smartcar TC264, Seekfree V3.4.5. */
#ifndef SMARTCAR_DRIVER_CONFIG_H
#define SMARTCAR_DRIVER_CONFIG_H

/* DRV8701E PH/EN. A PWM unit is 1/PWM_DUTY_MAX of one period. */
#define MOTOR_L_PH_PIN          P02_6               /* Left direction output. */
#define MOTOR_L_EN_PWM          ATOM0_CH7_P02_7     /* Left enable PWM. */
#define MOTOR_R_PH_PIN          P02_4               /* Right direction output. */
#define MOTOR_R_EN_PWM          ATOM0_CH5_P02_5     /* Right enable PWM. */
#define MOTOR_PWM_FREQ_HZ       17000               /* Both wheels, Hz. */
#define PWM_MAX                 5000                /* Maximum magnitude, duty units. */
#define MOTOR_L_REVERSE         1                   /* Invert the left command sign. */
#define MOTOR_R_REVERSE         1                   /* Invert the right command sign. */

/* GPT12 direction mode. Calibrate each wheel against measured travel. */
#define ENCODER_L_TIMER         TIM5_ENCODER           /* Left counter. */
#define ENCODER_L_COUNT_PIN     TIM5_ENCODER_CH1_P10_3 /* Left rising-edge input. */
#define ENCODER_L_DIR_PIN       TIM5_ENCODER_CH2_P10_1 /* Left direction input. */
#define ENCODER_R_TIMER         TIM6_ENCODER           /* Right counter. */
#define ENCODER_R_COUNT_PIN     TIM6_ENCODER_CH1_P20_3 /* Right rising-edge input. */
#define ENCODER_R_DIR_PIN       TIM6_ENCODER_CH2_P20_0 /* Right direction input. */
#define ENCODER_L_REVERSE       1                   /* Invert the left signed count. */
#define ENCODER_R_REVERSE       0                   /* Keep the right signed count. */
#define ENCODER_L_PULSES_PER_M  (11504.0f)          /* Left counts/m, provisional, > 0. */
#define ENCODER_R_PULSES_PER_M  (11504.0f)          /* Right counts/m, provisional, > 0. */
#define ENCODER_SAMPLE_MS       10                  /* Expected CCU60_CH0 period (ms) for speed conversion. Does not program PIT. */

/* BDS300 absolute PWM duty, on the same 0..PWM_DUTY_MAX scale. */
#define STEER_PWM_CHANNEL       ATOM1_CH1_P33_9     /* Steering signal output. */
#define STEER_PWM_FREQ_HZ       100                 /* PWM repetition rate, Hz. */
#define STEER_R_MIN             1330                /* Right mechanical limit, duty units. */
#define STEER_MIDDLE            1505                /* Current straight-ahead duty. */
#define STEER_L_MAX             1680                /* Left mechanical limit, duty units. */

/* Keys, switches and Hall input all use the board's pull-ups. */
#define KEY_1_PIN               P20_6               /* Button 1, active low. */
#define KEY_2_PIN               P20_7               /* Button 2, active low. */
#define KEY_3_PIN               P11_2               /* Button 3, active low. */
#define KEY_4_PIN               P11_3               /* Button 4, active low. */
#define KEY_SWITCH_1_PIN        P33_11              /* Active low, bit 1. */
#define KEY_SWITCH_2_PIN        P33_12              /* Active low, bit 0. */
#define KEY_HALL_PIN            P00_8               /* Raw logic level. */
#define KEY_SCAN_PERIOD_MS      10                  /* Expected CCU61_CH0 scan period (ms). Does not program PIT. */
#define KEY_DEBOUNCE_SAMPLES    3                   /* Consecutive samples per transition. */

/* LED pin identifiers from led.h, in the original physical order. */
#define LED1                    P20_9               /* Initial level low. */
#define LED2                    P20_8               /* Initial level high. */
#define LED3                    P21_5               /* Initial level low. */
#define LED4                    P21_4               /* Initial level high. */

#endif
