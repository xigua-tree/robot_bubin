#ifndef __OLED_H__
#define __OLED_H__

#include "main.h"

void oled_init(void);
void oled_write_cmd(uint8_t cmd);
void oled_write_data(uint8_t data);
void oled_fill(uint8_t data);
void oled_set_cursor(uint8_t x, uint8_t y);
void oled_show_char(uint8_t x, uint8_t y, uint8_t num, uint8_t size);
void oled_show_string(uint8_t x, uint8_t y, char *p, uint8_t size);
void oled_show_chinese(uint8_t x, uint8_t y, uint8_t N, uint8_t size);
void oled_show_image(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t *bmp);
void oled_task(void);

#endif
