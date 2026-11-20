/* Foreground-only bounded interval search on the test1 grayscale pipeline.
 * Image coordinates are pixels. No ground-plane projection or dynamic allocation. */
#include "vision_road.h"
#include "driver_config.h"
#include <string.h>

typedef struct { int y, weight, min_width, nominal, max_width; } scan_cfg_t;
#define SCAN(y,w,lo,nom,hi) {y,w,lo,nom,hi},
static const scan_cfg_t scan[] = { VISION_SCAN_ROWS(SCAN) };
#undef SCAN
typedef char check_rows[(VISION_ROW_COUNT >= 5 && VISION_ROW_COUNT <= 8 &&
                         sizeof(scan)/sizeof(scan[0]) == VISION_ROW_COUNT) ? 1 : -1];
vision_row_t vision_rows[VISION_ROW_COUNT];
static uint8 scan_mask[MT9V03X_W]; /* Foreground scratch for frames not sent to the screen. */
static int last_center[VISION_ROW_COUNT], last_width[VISION_ROW_COUNT];
static uint8 normal_streak, invalid_streak, loss_tripped;
static uint32 invalid_since_ms; /* Completion time of first frame in the current bad streak. */
static uint32 bad_count, loss_epoch;
static int previous_error;
static uint32 previous_ticks, previous_ms;
static uint8 derivative_ready;
static vision_state_t last_state;
static struct {
    uint8 confirmed, recovered, active, expired;
    uint32 entered_ms, geometry_ms, timeout_epoch;
    int detect_entry, detect_exit;
    int entry_x, exit_x; /* Last measured endpoints, never a cached steering command. */
} junction;
static int absolute(int x) { return x < 0 ? -x : x; }
int vision_row_y(unsigned i) { return scan[i].y; }
static void binarize_row(const uint8 *gray, uint8 *mask)
{
    unsigned x;
    for (x=0; x<MT9V03X_W; ++x)
        mask[x] = (VISION_WHITE_TRACK ? gray[x] > VISION_BINARY_THRESHOLD :
                                        gray[x] <= VISION_BINARY_THRESHOLD) ? 255U : 0U;
}
uint8 vision_config_valid(void)
{
    unsigned i;
    int weights = 0;
    for (i=0; i<VISION_ROW_COUNT; ++i)
    {
        const scan_cfg_t *c = &scan[i];
        if (c->y < 0 || c->y >= MT9V03X_H || (i && c->y >= scan[i-1].y) ||
            c->weight <= 0 || c->min_width <= 0 || c->min_width > c->nominal ||
            c->nominal > c->max_width || c->max_width >= MT9V03X_W) return 0;
        weights += c->weight;
    }
    return VISION_START_FRAMES > 0 && VISION_START_FRAMES <= 255 &&
        VISION_LOST_FRAMES > 0 && VISION_LOST_FRAMES <= 255 &&
        VISION_CROSS_CONFIRM > 0 && VISION_CROSS_CONFIRM <= 255 &&
        VISION_RECOVER_FRAMES > 0 && VISION_RECOVER_FRAMES <= 255 &&
        VISION_WIDE_PERCENT > 100 && VISION_PREDICT_MS < VISION_CROSS_LIMIT_MS &&
        VISION_CROSS_LIMIT_MS > 0 && VISION_FRAME_TIMEOUT_MS > 0 &&
        VISION_PD_MAX_GAP_MS > 0 && VISION_LOSS_MS > 0 &&
        VISION_MAX_SHIFT > 0 && VISION_EXIT_DELTA > 0 && weights <= 255 && VISION_TARGET_X >= 0 && VISION_TARGET_X < MT9V03X_W &&
        VISION_MIN_VALID_ROWS >= 2 && VISION_MIN_VALID_ROWS <= VISION_ROW_COUNT &&
        VISION_BINARY_THRESHOLD >= 0 && VISION_BINARY_THRESHOLD <= 255 &&
        (VISION_WHITE_TRACK == 0 || VISION_WHITE_TRACK == 1) &&
        (VISION_STEER_SIGN == 1 || VISION_STEER_SIGN == -1) &&
        VISION_STEER_MAX_DELTA >= 0 && STEER_MIDDLE-VISION_STEER_MAX_DELTA >= STEER_R_MIN &&
        STEER_MIDDLE+VISION_STEER_MAX_DELTA <= STEER_L_MAX;
}
void vision_road_reset(void)
{
    unsigned i;
    for (i=0; i<VISION_ROW_COUNT; ++i)
    {
        last_center[i] = -1;
        last_width[i] = scan[i].nominal;
        vision_rows[i].left = vision_rows[i].right = vision_rows[i].center = -1;
        vision_rows[i].valid = 0;
    }
    derivative_ready = 0;
    memset(&junction,0,sizeof(junction));
    last_state=VISION_LOST;
    normal_streak = invalid_streak = loss_tripped = 0;
    bad_count = loss_epoch = 0;
}

static vision_row_t choose_run(const uint8 *line, unsigned i, const vision_row_t *near)
{
    vision_row_t r = {0};
    int x=0, best=30000, second=30000, white=0;
    int expected = last_center[i] >= 0 ? last_center[i] : (near ? near->center : VISION_TARGET_X);
    int predicted = near ? (2*near->center+expected)/3 : expected;
    r.left=r.right=r.center=r.path_x=-1;
    while (x < MT9V03X_W)
    {
        int left,right,width,center,gap,score,from_run;
        while (x < MT9V03X_W && !line[x]) ++x;
        left=x;
        while (x < MT9V03X_W && line[x]) ++x;
        right=x-1; width=x-left;
        white += width;
        if (width < scan[i].min_width) continue;
        ++r.candidates;
        if (r.candidates==1) /* Retain a visible lone interval for diagnosis even if unassociated. */
        {
            r.left=(int16)left; r.right=(int16)right; r.width=(int16)width;
        }
        center=(left+right)/2;
        from_run = predicted < left ? left-predicted : (predicted > right ? predicted-right : 0);
        gap=0;
        if (near)
        {
            if (near->right < left) gap=left-near->right;
            else if (right < near->left) gap=near->left-right;
        }
        if (gap > VISION_OVERLAP_GAP || from_run > VISION_MAX_SHIFT) continue;
        score = 2*absolute(center-predicted) + absolute(width-last_width[i])/3 + 2*gap;
        if (score >= best) { if (score < second) second=score; continue; }
        second=best; best=score;
        r.left=(int16)left; r.right=(int16)right; r.width=(int16)width;
    }
    r.nearly_white = white*100 >= MT9V03X_W*95;
    if (r.left < 0) return r;
    r.left_seen=r.left>0; r.right_seen=r.right<MT9V03X_W-1;
    r.touches_edge=!r.left_seen || !r.right_seen;
    r.associated=best<30000;
    r.ambiguous=r.associated ? second-best <= VISION_AMBIGUITY : r.candidates>1;
    if (r.associated && !r.ambiguous && r.left_seen && r.right_seen && !r.nearly_white &&
        r.width <= scan[i].max_width && 100*r.width < VISION_WIDE_PERCENT*last_width[i])
    {
        r.center=(r.left+r.right)/2;
        r.path_x=r.center;
        r.valid=1;
    }
    return r;
}

static void measure(const uint8 image[MT9V03X_H][MT9V03X_W], vision_result_t *r,
                    uint8 mask[][MT9V03X_W])
{
    unsigned i;
    const vision_row_t *near = 0;
    int total=0, first=-1, last=-1;
    if (mask)
        for (i=0; i<MT9V03X_H; ++i) binarize_row(image[i],mask[i]);
    for (i=0; i<VISION_ROW_COUNT; ++i)
    {
        vision_row_t *row=&vision_rows[i];
        if (!mask) binarize_row(image[scan[i].y],scan_mask);
        *row=choose_run(mask ? mask[scan[i].y] : scan_mask,i,near);
        if (!row->valid) continue;
        near=row;
        if (first < 0) first=(int)i;
        last=(int)i;
        total+=row->center*scan[i].weight;
        r->weight+=(uint8)scan[i].weight;
        ++r->valid_rows;
    }
    r->valid = r->valid_rows >= VISION_MIN_VALID_ROWS && first <= 1 && last >= VISION_MID;
    r->center = r->weight ? (int16)((total+r->weight/2)/r->weight) : -1;
    if (first >= 0) r->trend=vision_rows[last].center-vision_rows[first].center;
    total=0;
    for (i=0; i<VISION_ROW_COUNT; ++i) total+=scan[i].weight;
    r->confidence=(uint8)(100*r->weight/total);
}

/* Interpolate through measured lane centers in image coordinates. */
static int path_at(int entry, int exit, int y)
{
    return entry+(exit-entry)*(scan[0].y-y)/(scan[0].y-scan[VISION_LAST].y);
}

static void use_path(int entry, int exit, vision_result_t *r)
{
    unsigned i;
    int total=0, weight=0;
    for (i=0;i<VISION_ROW_COUNT;++i)
    {
        vision_rows[i].path_x=(int16)path_at(entry,exit,scan[i].y);
        total+=scan[i].weight*vision_rows[i].path_x;
        weight+=scan[i].weight;
    }
    r->center=(int16)((total+weight/2)/weight);
    r->trend=(int16)(exit-entry);
    r->valid=1;
}

/* Two adjoining rows must widen on both sides of the candidate straight
 * path. Each compares against its own recent reliable width, not a near row. */
static int wide_band(int entry,int exit)
{
    int i, previous=0, count=0;
    for (i=1;i<VISION_LAST;++i)
    {
        const vision_row_t *row=&vision_rows[i];
        int x=path_at(entry,exit,scan[i].y);
        int wing=last_width[i]/2+VISION_WING_PX;
        int wide=row->width*100 >= last_width[i]*VISION_WIDE_PERCENT &&
                 row->associated && !row->ambiguous && row->left<=x-wing && row->right>=x+wing;
        if (wide && previous) ++count;
        previous=wide;
    }
    return count;
}

static void classify_junction(uint32 now, vision_result_t *r)
{
    const vision_row_t *entry=&vision_rows[0], *next=&vision_rows[1];
    const vision_row_t *exit=&vision_rows[VISION_LAST], *before=&vision_rows[VISION_LAST-1];
    int entrance=entry->valid && next->valid;
    int far=exit->valid && before->valid;
    int ex=entrance ? entry->center : VISION_TARGET_X;
    int out=far ? exit->center : (junction.active ? ex+junction.exit_x-junction.entry_x : ex);
    int broad=wide_band(ex,out);
    int supported=0;
    if (entrance && far && broad)
    {
        int projected=ex+(next->center-ex)*(scan[0].y-scan[VISION_LAST].y)/(scan[0].y-scan[1].y);
        supported=absolute(projected-out)<=VISION_EXIT_DELTA &&
                  absolute(before->center-path_at(ex,out,scan[VISION_LAST-1].y))<=VISION_EXIT_DELTA/2;
    }
    if (junction.active && now-junction.entered_ms >= VISION_CROSS_LIMIT_MS)
    {
        junction.active=0;
        junction.expired=1;
        ++junction.timeout_epoch;
    }
    if (junction.expired)
    {
        r->valid=0;
        r->state=VISION_LOST;
    }
    else if (junction.active)
    {
        int ordinary=r->valid && entrance && far && !broad &&
                     absolute(out-junction.exit_x)<=VISION_EXIT_DELTA;
        if (ordinary)
        {
            r->state=VISION_REACQUIRE;
            if (++junction.recovered>=VISION_RECOVER_FRAMES)
            {
                junction.active=0;
                junction.recovered=0;
                r->state=VISION_NORMAL;
            }
        }
        else
        {
            junction.recovered=0;
            r->state=VISION_CROSS_PASS;
            if (supported)
            {
                junction.entry_x=ex; junction.exit_x=out; junction.geometry_ms=now;
                use_path(ex,out,r);
            }
            else if (entrance && !far && broad && now-junction.geometry_ms < VISION_PREDICT_MS &&
                     absolute(ex-junction.entry_x)<=VISION_EXIT_DELTA && out>=0 && out<MT9V03X_W)
                use_path(ex,out,r); /* New entrance and last measured heading, time limited. */
            else { r->state=VISION_LOST; r->valid=0; }
        }
    }
    else if (broad && entrance)
    {
        r->state=VISION_CROSS_DETECT;
        r->valid=(uint8)(supported && VISION_CROSS_STRAIGHT);
        if (r->valid)
        {
            use_path(ex,out,r);
            if (junction.confirmed && (absolute(ex-junction.detect_entry)>VISION_EXIT_DELTA ||
                                       absolute(out-junction.detect_exit)>VISION_EXIT_DELTA))
                junction.confirmed=0;
            junction.detect_entry=ex; junction.detect_exit=out;
            if (++junction.confirmed>=VISION_CROSS_CONFIRM)
            {
                junction.active=1;
                junction.entered_ms=now;
                junction.geometry_ms=now;
                junction.entry_x=ex; junction.exit_x=out;
                junction.confirmed=0;
                r->state=VISION_CROSS_PASS;
            }
        }
        else junction.confirmed=0;
    }
    else
    {
        junction.confirmed=0;
        r->state=r->valid ? VISION_NORMAL : VISION_LOST;
    }
    r->cross_fault_epoch=junction.timeout_epoch;
    r->crossing=junction.active;
    r->cross_entered_ms=junction.entered_ms;
}

void vision_road_process(const uint8 image[MT9V03X_H][MT9V03X_W], uint32 now, float tick_seconds,
                         vision_result_t *r, uint8 mask[][MT9V03X_W])
{
    unsigned i;
    float output;
    float derivative=0.0f;
    measure(image,r,mask);
    classify_junction(now,r);
    if (last_state != r->state) derivative_ready=0;
    last_state=r->state;
    if (r->valid)
    {
        for (i=0; i<VISION_ROW_COUNT; ++i) if (r->state==VISION_NORMAL && vision_rows[i].valid)
        {
            last_center[i]=vision_rows[i].center;
            last_width[i]=(3*last_width[i]+vision_rows[i].width+2)/4;
        }
        invalid_streak=loss_tripped=0;
        if (r->state!=VISION_NORMAL) normal_streak=0;
        else if (normal_streak<255) ++normal_streak;
        r->error=r->center-VISION_TARGET_X;
        if (derivative_ready && now-previous_ms < VISION_PD_MAX_GAP_MS)
        {
            float dt=(r->completed_ticks-previous_ticks)*tick_seconds;
            if (dt>0.0f && dt < VISION_PD_MAX_GAP_MS*0.001f)
                derivative=(r->error-previous_error)/dt;
        }
        previous_ticks=r->completed_ticks; previous_ms=now; previous_error=r->error;
        derivative_ready=1;
        output=VISION_STEER_SIGN*(VISION_STEER_KP*r->error + VISION_STEER_KD*derivative);
        if (output>VISION_STEER_MAX_DELTA) output=VISION_STEER_MAX_DELTA;
        if (output<-VISION_STEER_MAX_DELTA) output=-VISION_STEER_MAX_DELTA;
        r->steer_offset=(int32)output;
        r->target_mps=(absolute(r->trend)>=VISION_CURVE_TREND || absolute(r->error)>=VISION_CURVE_ERROR)
                      ? VISION_CURVE_MPS : VISION_STRAIGHT_MPS;
        if (r->state!=VISION_NORMAL) r->target_mps=VISION_CROSS_MPS;
        if (VISION_STEER_ONLY) r->target_mps=0.0f;
    }
    else
    {
        derivative_ready=0;
        normal_streak=0;
        ++bad_count;
        if (!invalid_streak) invalid_since_ms=now;
        if (invalid_streak<VISION_LOST_FRAMES) ++invalid_streak;
        if (!loss_tripped && (invalid_streak>=VISION_LOST_FRAMES || now-invalid_since_ms>=VISION_LOSS_MS))
        {
            loss_tripped=1;
            ++loss_epoch;
            for (i=0;i<VISION_ROW_COUNT;++i)
            {
                last_center[i]=-1;
                if (!junction.active) last_width[i]=scan[i].nominal;
            }
        }
    }
    r->normal_streak=normal_streak;
    r->bad_count=bad_count; r->loss_epoch=loss_epoch;
}
