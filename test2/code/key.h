#ifndef SMARTCAR_KEY_H
#define SMARTCAR_KEY_H

#include "zf_common_typedef.h"
#include "driver_config.h"

/* One event per actuation: KEY1 on stable press, KEY2..4 on stable release. */
typedef enum
{
    KEY_EVENT_NONE = 0,   /* No event available for this reader. */
    KEY_EVENT_1 = 1,      /* Debounced press of P20_6, priority stop for control. */
    KEY_EVENT_2 = 2,      /* Release of P20_7. */
    KEY_EVENT_3 = 3,      /* Release of P11_2. */
    KEY_EVENT_4 = 4       /* Release of P11_3. */
} key_event_t;

typedef enum
{
    KEY_READER_CONTROL,   /* Vehicle control queue. */
    KEY_READER_UI         /* Independent UI queue. */
} key_reader_t;

#define KEY_QUEUE_CAPACITY 16  /* Pending events per reader; oldest lost on overflow. */

void key_input_init(void);                       /* Configure inputs before enabling PIT. */
void key_scan(void);                             /* Sample all inputs once each 10 ms. */
key_event_t key_read_event(key_reader_t reader); /* Control gets pending stop first, once. */
uint8 key_emergency_stop_requested(void);        /* Non-consuming latch query: 1 = stop. */
uint8 key_reset_emergency_stop(void);            /* Foreground after stop; 1 if cleared.
                                                  * Requires control delivery and all keys released. */
uint32 key_get_overflow_count(key_reader_t reader); /* Events displaced since init. */
int key_get_switches(void);                      /* SW1 bit 1, SW2 bit 0; active low. */
int key_get_hall_level(void);                    /* Last sampled Hall logic level, 0/1. */

/* Original public entry points. */
void usr_key_init(void);            /* Initialize the board inputs. */
void key_Handle(void);              /* Original PIT scan entry. */
int get_key_value(void);            /* Control reader, including the priority stop. */
int key_get_ui_value(void);         /* Pop the UI queue as the original integer type. */
int get_sw_value(void);             /* Switch bits from the last sample. */
int get_hall_value(void);           /* Hall logic level from the last sample. */
uint8 get_is_editing_flag(void);    /* Original unused driver flag, always zero. */
void set_key_value(uint8 value);    /* 0 clears FIFOs, 1 requests stop; neither resets the latch. */

/* Define only for the old ui.c translation unit if it still calls get_key_value(). */
#ifdef SMARTCAR_UI_KEY_READER
#define get_key_value key_get_ui_value
#endif

#endif
