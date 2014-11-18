#pragma once

#include <stdint.h>

void Ssd1306_HAL_Init();
uint8_t Ssd1306_HAL_SpiTransfer(uint8_t data);
void Ssd1306_HAL_ChipSelect();
void Ssd1306_HAL_ChipDeselect();
void Ssd1306_HAL_DataSelect();
void Ssd1306_HAL_ComandSelect();
void Ssd1306_HAL_ResetEnable();
void Ssd1306_HAL_ResetDisnable();
