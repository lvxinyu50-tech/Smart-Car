/* IPS200 portrait, 30 columns of 8x16 text. All work runs on CPU0 foreground. */
#include "test_display.h"
#include "test_mode.h"
#include "test_mode_config.h"
#include "vision_track.h"
#include "vision_display.h"
#include "zf_device_ips200.h"
#include "zf_common_font.h"
#include <stdio.h>
#include <string.h>

#define COLUMNS 30
#define TEXT_ROWS 20
static char wanted[TEXT_ROWS][COLUMNS+1], drawn[TEXT_ROWS][COLUMNS+1];
static unsigned text_cursor; /* Next row to compare after telemetry is formatted. */
static const vision_debug_frame_t *painting;
static uint16 next_y=MT9V03X_H;
static uint8 started, have_id, has_picture, gray_page;
static uint32 last_refresh, page_since, mode_since, frame_id, session;
static test_state_t previous_mode;

/* Only send the changed span of one text row per main-loop pass. */
static void flush_text_row(void)
{
    unsigned row,first,last,len;
    char piece[COLUMNS+1];
    while (text_cursor<TEXT_ROWS)
    {
        row=text_cursor++;
        if (!strcmp(wanted[row],drawn[row])) continue;
        len=(unsigned)strlen(wanted[row]);
        for (first=len; first<COLUMNS; ++first) wanted[row][first]=' ';
        wanted[row][COLUMNS]='\0';
        for (first=0; first<COLUMNS && wanted[row][first]==drawn[row][first]; ++first) {}
        if (first==COLUMNS) continue;
        last=COLUMNS;
        while (last>first && wanted[row][last-1]==drawn[row][last-1]) --last;
        memcpy(piece,&wanted[row][first],last-first);
        piece[last-first]='\0';
        ips200_show_string((uint16)(8*first),(uint16)(16*row),piece);
        memcpy(drawn[row],wanted[row],COLUMNS+1);
        return;
    }
}

static long mm_per_s(float mps)
{
    return (long)(mps*1000.0f + (mps<0.0f ? -0.5f : 0.5f));
}

static void telemetry(uint32 now,const vision_debug_frame_t *f,uint8 fresh,const test_status_t *s)
{
    static const char *const modes[]={"IDLE","OPEN LOOP","SPEED PI","VISION","FAULT"};
    static const char *const states[]={"NORMAL","CROSS DETECT","CROSS PASS","REACQUIRE","VISION LOST"};
    static const char *const faults[]={"NONE","KEY1","COAST TIME","REPORTED","CONFIG","CAM INIT",
                                       "FRAME TIME","VISION LOST","CROSS TIME"};
    const vision_result_t *r=f ? &f->result : NULL;
    const char *phase=s->waiting ? "WAIT" : s->vision_hold ? "HOLD" :
                      (s->state==TEST_IDLE || s->state==TEST_FAULT ? "STOP" : "RUN");
    const char *cam="CAMERA WAIT";
    const char *geometry="NO FRAME";
    unsigned i;
    if (!vision_track_camera_ok() || (!fresh && (f || s->fault==TEST_FRAME_TIMEOUT ||
                                                 now-mode_since>=VISION_FRAME_TIMEOUT_MS)))
        cam="CAMERA ERROR";
    else if (fresh) cam="CAMERA OK";
    if (fresh) geometry=states[r->state];
    else if (s->fault==TEST_LINE_LOST || s->fault==TEST_CROSS_TIMEOUT) geometry="VISION LOST";
    snprintf(wanted[0],COLUMNS+1,"%s %s   %s",modes[s->state],phase,gray_page?"A GRAY":"B MASK");
    snprintf(wanted[1],COLUMNS+1,"%s  %s",cam,geometry);
    if (r)
        snprintf(wanted[10],COLUMNS+1,"FR %lu AGE %lums",(unsigned long)r->capture_id,
                 (unsigned long)(now-r->completed_ms));
    else strcpy(wanted[10],"FR -- AGE --");
    if (fresh)
        snprintf(wanted[11],COLUMNS+1,"THR %d %s ROWS %u/%u Q%u W%u",VISION_BINARY_THRESHOLD,
                 VISION_WHITE_TRACK?"HI":"LO",(unsigned)r->valid_rows,(unsigned)VISION_ROW_COUNT,
                 (unsigned)r->confidence,(unsigned)r->weight);
    else snprintf(wanted[11],COLUMNS+1,"THR %d %s ROWS --",VISION_BINARY_THRESHOLD,VISION_WHITE_TRACK?"HI":"LO");
    for (i=0; fresh && i<VISION_ROW_COUNT && !f->rows[i].valid; ++i) {}
    if (fresh && i<VISION_ROW_COUNT)
        snprintf(wanted[12],COLUMNS+1,"Y%d L%d R%d CX%d",vision_row_y(i),(int)f->rows[i].left,
                 (int)f->rows[i].right,(int)r->center);
    else strcpy(wanted[12],"Y-- L-- R-- CX--");
    if (fresh && r->valid)
        snprintf(wanted[13],COLUMNS+1,"ERR %+d SERVO %ld/%ld",(int)r->error,
                 (long)(STEER_MIDDLE+r->steer_offset),(long)s->servo_duty);
    else snprintf(wanted[13],COLUMNS+1,"ERR -- SERVO --/%ld",(long)s->servo_duty);
    snprintf(wanted[14],COLUMNS+1,"TGT %ld RAMP %ld mm/s",mm_per_s(s->target_mps),mm_per_s(s->ramped_mps));
    snprintf(wanted[15],COLUMNS+1,"V L %ld R %ld mm/s",mm_per_s(s->encoder.left.speed_mps),mm_per_s(s->encoder.right.speed_mps));
    snprintf(wanted[16],COLUMNS+1,"PWM REQ %d / %d",s->left_motor.requested_pwm,s->right_motor.requested_pwm);
    snprintf(wanted[17],COLUMNS+1,"PWM OUT %d / %d",s->left_motor.applied_pwm,s->right_motor.applied_pwm);
    snprintf(wanted[18],COLUMNS+1,"ESTOP %s  RESET %s",s->estop?"LATCHED":"CLEAR",s->reset_ready?"YES":"NO");
    if (s->fault) snprintf(wanted[19],COLUMNS+1,"FAULT %s",faults[s->fault]);
    else if (fresh && !gray_page && (s->state==TEST_VISION_TRACK || s->state==TEST_IDLE))
        snprintf(wanted[19],COLUMNS+1,"NEAR %d MID %d FAR %d",(int)f->rows[0].center,
                 (int)f->rows[VISION_MID].center,(int)f->rows[VISION_LAST].center);
    else snprintf(wanted[19],COLUMNS+1,"ENC/10ms %d / %d",(int)s->encoder.left.delta_pulses,(int)s->encoder.right.delta_pulses);
}

void test_display_init(void)
{
    static const char *const key[]={"L  ","R  ","MID","REF","BAD"};
    static const uint16 colors[]={RGB565_GREEN,RGB565_RED,RGB565_YELLOW,RGB565_BLUE,VISION_INVALID_COLOR};
    unsigned i;
    ips200_clear();
    for (i=0;i<sizeof(colors)/sizeof(colors[0]);++i)
    {
        ips200_set_color(colors[i],RGB565_BLACK);
        ips200_show_string(216,(uint16)(48+i*16),key[i]);
    }
    ips200_set_color(IPS200_DEFAULT_PENCOLOR,IPS200_DEFAULT_BGCOLOR);
    memset(drawn,' ',sizeof(drawn));
    memset(wanted,' ',sizeof(wanted));
    for (i=0;i<TEXT_ROWS;++i) drawn[i][COLUMNS]=wanted[i][COLUMNS]='\0';
    test_display_task(test_mode_time_ms());
}

void test_display_task(uint32 now)
{
    const vision_debug_frame_t *f=vision_debug_latest();
    uint8 fresh=f && now-f->result.completed_ms<VISION_FRAME_TIMEOUT_MS;
    uint8 changed=f && (!have_id || f->result.capture_id!=frame_id || f->result.generation!=session);
    uint8 expired=has_picture && !fresh;
    if (expired || (next_y>=MT9V03X_H && (!started || changed || now-last_refresh>=TEST_DISPLAY_MS)))
    {
        test_status_t s=test_mode_status();
        if (!started || previous_mode!=s.state || (f && have_id && f->result.generation!=session))
        {
            gray_page=0;
            page_since=mode_since=now;
            previous_mode=s.state;
        }
        else if (changed && fresh && (s.state==TEST_VISION_TRACK || s.state==TEST_IDLE) &&
                 now-page_since>=VISION_PAGE_MS)
        {
            gray_page=!gray_page;
            page_since=now;
        }
        telemetry(now,f,fresh,&s);
        text_cursor=0;
        if (changed)
        {
            session=f->result.generation;
            frame_id=f->result.capture_id;
            have_id=1;
        }
        if ((!started || expired) || (changed && fresh))
        {
            painting=fresh ? f : NULL;
            has_picture=fresh;
            next_y=0;
        }
        /* No new snapshot request until all strips of a live picture have been sent. */
        if (!changed) vision_debug_request();
        last_refresh=now;
        started=1;
    }
    flush_text_row();
    if (next_y<MT9V03X_H)
    {
        vision_display_strip(painting,next_y,gray_page);
        next_y+=VISION_STRIP_ROWS;
    }
}
