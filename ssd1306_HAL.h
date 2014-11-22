#pragma once

#include <stdint.h>

#define SSD1306_LCDWIDTH                  128
#define SSD1306_LCDHEIGHT                 64

void Ssd1306_HAL_Init();
void Ssd1306_HAL_Process(void *param);


void Ssd1306_HAL_Lock();
void Ssd1306_HAL_Unlock();

uint8_t Ssd1306_HAL_SpiTransfer(uint8_t data);
void Ssd1306_HAL_ChipSelect();
void Ssd1306_HAL_ChipDeselect();
void Ssd1306_HAL_DataSelect();
void Ssd1306_HAL_ComandSelect();
void Ssd1306_HAL_ResetEnable();
void Ssd1306_HAL_ResetDisnable();
void Ssd1306_HAL_Start();
void Ssd1306_HAL_Stop();
