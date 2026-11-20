/* TC264 chassis bench. CPU0 foreground owns input and display; CCU60 owns drive. */
#include "zf_common_headfile.h"
#include "motor.h"
#include "encoder.h"
#include "steer.h"
#include "key.h"
#include "led.h"
#include "vision_track.h"
#include "test_mode/test_mode.h"
#include "test_mode/test_mode_config.h"
#include "test_mode/test_display.h"

#pragma section all "cpu0_dsram"

int core0_main(void)
{
    uint32 last_display_ms = 0;
    clock_init();
    debug_init();

    motor_init();
    encoder_init();
    steer_init();
    key_input_init();
    led_init();
    test_mode_init();                  /* IDLE, PWM=0, centered, PI stopped. */

    ips200_init(IPS200_TYPE_SPI);
    vision_track_init();               /* Motors are off; SDK selects SCCB or UART. */
    test_display_init();
    cpu_wait_event_ready();
    pit_ms_init(CCU60_CH0, TEST_TICK_MS);
    pit_ms_init(CCU61_CH0, KEY_SCAN_PERIOD_MS);
    while (TRUE)
    {
        uint32 now;
        test_mode_poll();
        vision_track_task();           /* Processes one owned full frame, no actuator writes. */
        now = test_mode_time_ms();
        if ((uint32)(now - last_display_ms) >= TEST_DISPLAY_MS)
        {
            last_display_ms = now;
            test_display_refresh();
        }
    }
}
#pragma section all restore
