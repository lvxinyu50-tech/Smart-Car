#include "key.h"
#include "zf_common_interrupt.h"
#include "zf_driver_gpio.h"
#include <stdint.h>

#if (KEY_DEBOUNCE_SAMPLES < 1) || (KEY_DEBOUNCE_SAMPLES > 255)
#error Key debounce sample count must fit uint8
#endif

static const gpio_pin_enum key_pins[] = {KEY_1_PIN, KEY_2_PIN, KEY_3_PIN, KEY_4_PIN};
#define BUTTON_COUNT (sizeof(key_pins) / sizeof(key_pins[0])) /* Physical inputs. */
#define KEY_READER_COUNT 2U /* One queue each for control and UI. */

typedef struct
{
    uint8 stable_level;   /* Last debounced GPIO level. */
    uint8 changed_scans;  /* Consecutive samples different from stable_level. */
} debounce_t;

typedef struct
{
    uint8 events[KEY_QUEUE_CAPACITY]; /* Oldest entry at head. */
    uint8 head;                      /* Index of next event to return. */
    uint8 count;                     /* Number of unread events. */
    uint32 overflows;                /* Saturating count of displaced events. */
} event_queue_t;

static volatile debounce_t buttons[BUTTON_COUNT]; /* ISR writes; foreground checks before reset. */
static volatile event_queue_t queues[KEY_READER_COUNT]; /* Indexed by key_reader_t, CPU0 only. */
static volatile uint8 switches;             /* Active-low switch bits, sampled together. */
static volatile uint8 hall_level;           /* Cached raw Hall level. */

/* CPU0 owns the latch. Reading the priority notification advances PENDING to
 * DELIVERED; only an explicit reset can return it to READY. */
typedef enum { STOP_READY, STOP_PENDING, STOP_DELIVERED } stop_state_t;
static volatile stop_state_t stop_state = STOP_READY;

/* Lock held by caller. Overwrite only this reader's oldest entry on a full queue. */
static void push_event(volatile event_queue_t *queue, uint8 event)
{
    uint8 tail;  /* Slot to receive the new event. */
    if (queue->count == KEY_QUEUE_CAPACITY)
    {
        queue->head = (uint8)((queue->head + 1U) % KEY_QUEUE_CAPACITY);
        --queue->count;
        if (queue->overflows < UINT32_MAX)
            ++queue->overflows;
    }
    tail = (uint8)((queue->head + queue->count) % KEY_QUEUE_CAPACITY);
    queue->events[tail] = event;
    ++queue->count;
}

/* KEY2/KEY3 are the existing control reader's two start/mode commands. */
static int is_mode_event(uint8 event)
{
    return event == KEY_EVENT_2 || event == KEY_EVENT_3;
}

/* Invalidate pre-stop mode commands, preserving e.g. KEY4 and FIFO order.
 * At most KEY_QUEUE_CAPACITY entries, under the same lock as publication. */
static void discard_control_modes(void)
{
    volatile event_queue_t *queue = &queues[KEY_READER_CONTROL];
    uint8 i, kept = 0;
    for (i = 0; i < queue->count; ++i)
    {
        uint8 event = queue->events[(queue->head + i) % KEY_QUEUE_CAPACITY];
        if (!is_mode_event(event))
        {
            queue->events[(queue->head + kept) % KEY_QUEUE_CAPACITY] = event;
            ++kept;
        }
    }
    queue->count = kept;
}

/* Caller holds the CPU0 interrupt lock. The stop notification has its own slot. */
static void publish_event(uint8 event)
{
    if (event == KEY_EVENT_1)
    {
        if (stop_state == STOP_READY)
            discard_control_modes();
        stop_state = STOP_PENDING;
    }
    else if (stop_state == STOP_READY || !is_mode_event(event))
        push_event(&queues[KEY_READER_CONTROL], event);
    push_event(&queues[KEY_READER_UI], event);
}

/* These inputs retain their original raw sampling semantics. */
static void sample_aux_inputs(void)
{
    switches = (uint8)(((!gpio_get_level(KEY_SWITCH_1_PIN)) << 1)
                       | (!gpio_get_level(KEY_SWITCH_2_PIN)));
    hall_level = gpio_get_level(KEY_HALL_PIN);
}

void key_input_init(void)
{
    unsigned int i;  /* Physical key or reader index. */
    gpio_init(KEY_SWITCH_1_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY_SWITCH_2_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    for (i = 0; i < BUTTON_COUNT; ++i)
    {
        gpio_init(key_pins[i], GPI, GPIO_HIGH, GPI_PULL_UP);
        if (stop_state == STOP_READY)
        {
            buttons[i].stable_level = GPIO_HIGH;
            buttons[i].changed_scans = 0;
        }
    }
    gpio_init(KEY_HALL_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    sample_aux_inputs();
    for (i = 0; i < KEY_READER_COUNT; ++i)
    {
        queues[i].head = 0;
        queues[i].count = 0;
        queues[i].overflows = 0;
    }
}

void key_scan(void)
{
    unsigned int i; /* KEY1 is sampled first, then KEY2..4 in board order. */
    for (i = 0; i < BUTTON_COUNT; ++i)
    {
        volatile debounce_t *button = &buttons[i];
        uint8 level = gpio_get_level(key_pins[i]);
        if (level == button->stable_level)
        {
            button->changed_scans = 0;
        }
        else if (++button->changed_scans == KEY_DEBOUNCE_SAMPLES)
        {
            button->stable_level = level;
            button->changed_scans = 0;
            /* KEY1 fires on stable press. The other buttons fire on release. */
            if (level == ((i == 0) ? GPIO_LOW : GPIO_HIGH))
            {
                uint32 mask = interrupt_global_disable();
                publish_event((uint8)(KEY_EVENT_1 + i));
                interrupt_global_enable(mask);
            }
        }
    }
    sample_aux_inputs();
}

uint8 key_emergency_stop_requested(void)
{
    return (uint8)(stop_state != STOP_READY);
}

uint8 key_reset_emergency_stop(void)
{
    unsigned int i;
    uint8 cleared = 0;
    uint32 mask = interrupt_global_disable();
    /* Foreground calls this after executing the stop. Refuse while a key is
     * down, changing, or newly pressed since the most recent scan. */
    if (stop_state == STOP_DELIVERED)
    {
        for (i = 0; i < BUTTON_COUNT; ++i)
        {
            if (buttons[i].stable_level != GPIO_HIGH || buttons[i].changed_scans != 0 ||
                gpio_get_level(key_pins[i]) != GPIO_HIGH)
                break;
        }
        if (i == BUTTON_COUNT)
        {
            stop_state = STOP_READY;
            cleared = 1;
        }
    }
    interrupt_global_enable(mask);
    return cleared;
}

key_event_t key_read_event(key_reader_t reader)
{
    uint32 mask;                     /* Previous interrupt state. */
    uint8 event = KEY_EVENT_NONE;     /* Empty until an unread event is found. */
    volatile event_queue_t *queue;
    if (reader != KEY_READER_CONTROL && reader != KEY_READER_UI)
        return KEY_EVENT_NONE;
    mask = interrupt_global_disable();
    queue = &queues[reader];
    if (reader == KEY_READER_CONTROL && stop_state == STOP_PENDING)
    {
        stop_state = STOP_DELIVERED;
        event = KEY_EVENT_1;
    }
    else if (queue->count != 0)
    {
        event = queue->events[queue->head];
        queue->head = (uint8)((queue->head + 1U) % KEY_QUEUE_CAPACITY);
        --queue->count;
    }
    interrupt_global_enable(mask);
    return (key_event_t)event;
}

uint32 key_get_overflow_count(key_reader_t reader)
{
    if (reader != KEY_READER_CONTROL && reader != KEY_READER_UI)
        return 0;
    return queues[reader].overflows; /* Naturally aligned uint32 on CPU0. */
}

void set_key_value(uint8 value)
{
    /* Replace ordinary pending data only. ID 1 explicitly requests a stop. */
    uint32 mask = interrupt_global_disable();
    queues[KEY_READER_CONTROL].count = 0;
    queues[KEY_READER_UI].count = 0;
    if (value != 0)
        publish_event(value);
    interrupt_global_enable(mask);
}

int key_get_switches(void)       { return switches; }
int key_get_hall_level(void)     { return hall_level; }
void usr_key_init(void)          { key_input_init(); }
void key_Handle(void)            { key_scan(); }
int get_key_value(void)          { return (int)key_read_event(KEY_READER_CONTROL); }
int key_get_ui_value(void)       { return (int)key_read_event(KEY_READER_UI); }
int get_sw_value(void)           { return key_get_switches(); }
int get_hall_value(void)         { return key_get_hall_level(); }
uint8 get_is_editing_flag(void)  { return 0; }
