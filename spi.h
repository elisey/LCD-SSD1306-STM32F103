#pragma once

#include "stm32f10x.h"
#include <stdint.h>

typedef enum	{
	spi_clock_mode_idle_low_1_edge = 0,
	spi_clock_mode_idle_high_1_edge,
	spi_clock_mode_idle_low_2_edge,
	spi_clock_mode_idle_high_2_edge
} spi_clock_mode_t;

typedef enum	{
	spi_first_bit_msb = 0,
	spi_first_bit_lsb
} spi_first_bit_t;

typedef enum	{
	spi_prescaler_2 = 0,
	spi_prescaler_4,
	spi_prescaler_8,
	spi_prescaler_16,
	spi_prescaler_32,
	spi_prescaler_64,
	spi_prescaler_128,
	spi_prescaler_256
} spi_prescaler_t;

typedef struct	{
	SPI_TypeDef *SPIx;
} spi_t;

extern spi_t spi1, spi2, spi3;

void SpiDriver_Init(
		spi_t* this,
		spi_clock_mode_t spi_clock_mode,
		spi_first_bit_t spi_first_bit,
		spi_prescaler_t spi_prescaler);

uint8_t SpiDriver_BlockingTransfer(spi_t* this, uint8_t data);
