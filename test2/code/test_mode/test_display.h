#ifndef TEST2_DISPLAY_H
#define TEST2_DISPLAY_H
#include "zf_common_typedef.h"
void test_display_init(void);      /* Foreground only, after IPS200 SPI initialization. */
void test_display_task(uint32 now_ms); /* Called every foreground pass; paced internally. */
#endif
