/* Render already selected geometry; this module performs no road search. */
#include "vision_display.h"
#include "zf_device_ips200.h"
#include "zf_common_font.h"

#define IMAGE_X ((240 - MT9V03X_W) / 2)
#define IMAGE_Y 32
#if MT9V03X_W > 240 || IMAGE_Y + MT9V03X_H > 160
#error Camera preview does not fit the IPS200 portrait image area
#endif
static uint16 strip[VISION_STRIP_ROWS][MT9V03X_W];

static uint16 gray565(uint8 g)
{
    return (uint16)(((uint16)(g & 0xf8U) << 8) | ((uint16)(g & 0xfcU) << 3) | (g >> 3));
}

static void mark(uint16 *line, int x, int radius, uint16 color)
{
    int i;
    for (i=x-radius; i<=x+radius; ++i)
        if (i>=0 && i<MT9V03X_W) line[i]=color;
}

/* Between sample rows the colored line only connects adjacent accepted endpoints.
 * A rejected row breaks the boundary trace. Yellow cross paths use use_path() samples. */
static int between(int near_x, int far_x, int y, int near_y, int far_y)
{
    return near_x + (far_x-near_x)*(near_y-y)/(near_y-far_y);
}

static void contours(uint16 *line, int y, const vision_debug_frame_t *f)
{
    unsigned i;
    if ((y % 6) < 4) line[VISION_TARGET_X]=RGB565_BLUE;
    for (i=0; i<VISION_ROW_COUNT; ++i)
    {
        const vision_row_t *row=&f->rows[i];
        int sy=vision_row_y(i);
        if (y==sy)
        {
            if (row->valid)
            {
                mark(line,row->left,2,RGB565_GREEN);
                mark(line,row->right,2,RGB565_RED);
            }
            else
            {
                mark(line,1,1,VISION_INVALID_COLOR); /* Even a row without any candidate is visible. */
                if (row->left>=0) mark(line,row->left,2,VISION_INVALID_COLOR);
                if (row->right>=0) mark(line,row->right,2,VISION_INVALID_COLOR);
            }
            if (f->result.valid && row->path_x>=0) mark(line,row->path_x,2,RGB565_YELLOW);
            return;
        }
        if (i && y>sy && y<vision_row_y(i-1))
        {
            const vision_row_t *near=&f->rows[i-1];
            int ny=vision_row_y(i-1);
            if (near->valid && row->valid)
            {
                mark(line,between(near->left,row->left,y,ny,sy),0,RGB565_GREEN);
                mark(line,between(near->right,row->right,y,ny,sy),0,RGB565_RED);
            }
            if (f->result.valid && near->path_x>=0 && row->path_x>=0)
            {
                /* Dashed between samples where junction geometry bridges missing boundaries. */
                if ((near->valid && row->valid) || y%4<2)
                    mark(line,between(near->path_x,row->path_x,y,ny,sy),0,RGB565_YELLOW);
            }
            return;
        }
    }
}

void vision_display_strip(const vision_debug_frame_t *frame, uint16 y, uint8 gray_page)
{
    unsigned j,x;
    uint16 count;
    if (y>=MT9V03X_H) return;
    count=(uint16)(MT9V03X_H-y);
    if (count>VISION_STRIP_ROWS) count=VISION_STRIP_ROWS;
    for (j=0; j<count; ++j)
    {
        uint16 *line=strip[j];
        for (x=0; x<MT9V03X_W; ++x)
        {
            if (!frame) line[x]=RGB565_BLACK;
            else if (gray_page) line[x]=gray565(frame->gray[y+j][x]);
            else line[x]=frame->binary[y+j][x] ? RGB565_WHITE : RGB565_BLACK;
        }
        if (frame && !gray_page) contours(line,(int)(y+j),frame);
    }
    ips200_show_rgb565_image(IMAGE_X,(uint16)(IMAGE_Y+y),strip[0],MT9V03X_W,count,MT9V03X_W,count,0);
}
