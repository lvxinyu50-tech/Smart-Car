#ifndef CODE_LAP_COUNTER_H_
#define CODE_LAP_COUNTER_H_

/* Counts returns to the single start/finish zebra crossing. */
#define LAP_TARGET_COUNT          2u
#define LAP_START_IGNORE_FRAMES 300u
#define LAP_CLEAR_FRAMES         10u

extern volatile unsigned char lap_count;
/* Legacy stop flag: set only when the target lap count is reached. */
extern volatile unsigned char banmaxian_on;

/* CPU0 requests a new run; CPU1 owns and resets the counter state. */
void lap_counter_request_start(void);
unsigned char lap_counter_start_pending(void);
void lap_counter_update(unsigned char running,
                        unsigned char detection_valid,
                        unsigned char zebra_present);

#endif
