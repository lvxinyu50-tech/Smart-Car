#include "test_display.h"
#include "test_mode.h"
#include "vision_track.h"
#include "zf_device_ips200.h"
#include "zf_common_font.h"

/* 240 x 320 portrait, 8 x 16 font. The preview uses the three foreground
 * binary rows, never the live DMA buffer. All SPI work is outside PIT. */
void test_display_init(void)
{
    ips200_clear();
    ips200_show_string(0, 16, "ESTOP          RESET");
    ips200_show_string(0, 32, "FAULT");
    ips200_show_string(0, 48, "CAM           FR");
    ips200_show_string(0, 64, "THR           AGE ms");
    ips200_show_string(0, 80, "ROW     LEFT   RIGHT  CENTER");
    ips200_show_string(0, 96, "NEAR");
    ips200_show_string(0, 112, "MID");
    ips200_show_string(0, 128, "FAR");
    ips200_show_string(0, 144, "ERR           SERVO");
    ips200_show_string(88, 160, "LEFT     RIGHT");
    ips200_show_string(0, 176, "PWM req");
    ips200_show_string(0, 192, "PWM apply");
    ips200_show_string(0, 208, "ENC /10ms");
    ips200_show_string(0, 224, "V m/s");
    ips200_show_string(0, 240, "SET m/s");
    ips200_show_string(0, 256, "RAMP m/s");
    ips200_show_int(32, 64, VISION_BINARY_THRESHOLD, 3);
    ips200_show_string(72, 64, VISION_WHITE_TRACK ? "W" : "B");
    test_display_refresh();
}

static void draw_row(uint8 i)
{
    uint16 y = 96 + 16 * i;
    uint16 strip = 276 + 14 * i;
    ips200_show_int(64, y, track_left[i], 3);
    ips200_show_int(120, y, track_right[i], 3);
    ips200_show_int(176, y, track_center[i], 3);
    ips200_show_gray_image(0, strip, track_binary[i], MT9V03X_W, 1, MT9V03X_W, 4, 0);
    if (track_row_valid[i])
    {
        ips200_draw_line(track_left[i], strip, track_left[i], strip + 3, RGB565_RED);
        ips200_draw_line(track_right[i], strip, track_right[i], strip + 3, RGB565_RED);
        ips200_draw_line(track_center[i], strip, track_center[i], strip + 3, RGB565_GREEN);
    }
}

void test_display_refresh(void)
{
    static const char *const modes[] = {"IDLE        ", "OPEN LOOP   ", "CLOSED LOOP ",
                                        "VISION TRACK", "FAULT       "};
    static const char *const faults[] = {"NONE      ", "KEY1      ", "COAST TIME",
                                         "REPORTED  ", "CONFIG    ", "CAM INIT  ",
                                         "FRAME TIME", "LINE LOST "};
    test_status_t s = test_mode_status();
    vision_result_t r = vision_track_result();
    uint32 age = test_mode_time_ms() - r.started_ms;
    const char *phase = "STOPPED ";
    const char *camera = "NO FRAME";
    uint8 i;
    if (s.waiting) phase = "WAIT 0  ";
    else if (s.vision_hold) phase = "HOLD    ";
    else if (s.state != TEST_IDLE && s.state != TEST_FAULT) phase = "RUN     ";
    if (!vision_track_camera_ok()) camera = "INIT ERR";
    else if (r.has_frame)
        camera = (age >= VISION_FRAME_TIMEOUT_MS) ? "STALE   " : (r.valid ? "TRACK OK" : "NO LINE ");
    ips200_show_string(0, 0, modes[s.state]);
    ips200_show_string(128, 0, phase);
    ips200_show_string(48, 16, s.estop ? "LATCHED" : "CLEAR  ");
    ips200_show_string(176, 16, s.reset_ready ? "YES" : "NO ");
    ips200_show_string(56, 32, faults[s.fault]);
    ips200_show_string(32, 48, camera);
    ips200_show_uint(136, 48, vision_track_get_frame_count(), 10);
    ips200_show_int(168, 64, !r.has_frame ? -1 : (int32)(age < 9999U ? age : 9999U), 4);
    for (i = 0; i < VISION_ROW_COUNT; ++i) draw_row(i);
    ips200_show_int(40, 144, track_error, 3);
    ips200_show_int(160, 144, s.servo_duty, 4);
    ips200_show_int(88, 176, s.left_motor.requested_pwm, 5);
    ips200_show_int(160, 176, s.right_motor.requested_pwm, 5);
    ips200_show_int(88, 192, s.left_motor.applied_pwm, 5);
    ips200_show_int(160, 192, s.right_motor.applied_pwm, 5);
    ips200_show_int(88, 208, s.encoder.left.delta_pulses, 5);
    ips200_show_int(160, 208, s.encoder.right.delta_pulses, 5);
    ips200_show_float(88, 224, s.encoder.left.speed_mps, 2, 3);
    ips200_show_float(160, 224, s.encoder.right.speed_mps, 2, 3);
    ips200_show_float(88, 240, s.target_mps, 2, 3);
    ips200_show_float(160, 240, s.target_mps, 2, 3);
    ips200_show_float(88, 256, s.ramped_mps, 2, 3);
    ips200_show_float(160, 256, s.ramped_mps, 2, 3);
}
