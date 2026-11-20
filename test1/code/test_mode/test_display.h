#ifndef TEST1_DISPLAY_H
#define TEST1_DISPLAY_H
void test_display_init(void);      /* Foreground only, after IPS200 SPI initialization. */
void test_display_refresh(void);   /* Foreground only, every 200 ms. */
#endif
