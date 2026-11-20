#ifndef TEST2_VISION_DISPLAY_H
#define TEST2_VISION_DISPLAY_H
#include "vision_debug.h"

#define VISION_STRIP_ROWS 6 /* At most one 188 x 6 SPI transfer per foreground pass. */
#define VISION_INVALID_COLOR 0xFD20U /* RGB565 orange: rejected boundary or missing row. */
void vision_display_strip(const vision_debug_frame_t *frame, uint16 y, uint8 gray_page);
#endif
