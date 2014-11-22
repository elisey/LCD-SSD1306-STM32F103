#pragma once

#include <stdint.h>

#define ssd1306REFRESH_RATE_MS	20

void Ssd1306_Task(void *param);
void Ssd1306_HAL_Lock();
void Ssd1306_HAL_Unlock();
void Ssd1306_DrawPixel(uint8_t x, uint8_t y);
void Ssd1306_ClearPixel(uint8_t x, uint8_t y);
uint8_t Ssd1306_GetPixel(uint8_t x, uint8_t y);
void Ssd1306_ClearScreen(void);
