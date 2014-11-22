#include "spi.h"

#include "debug.h"

spi_t spi1 = {SPI1};
spi_t spi2 = {SPI2};
spi_t spi3 = {SPI3};

#define IS_VALID_SPI_OBJECT_POINTER(x)	( ( (x) == &spi1) || ( (x) == &spi2) || ((x) == &spi3) )

static void prv_rccInit(spi_t* this);
static void prv_gpioInit(spi_t* this);
static void prv_spiInit(
		spi_t *this,
		spi_clock_mode_t spi_clock_mode,
		spi_first_bit_t spi_first_bit,
		spi_prescaler_t spi_prescaler);

void SpiDriver_Init(
		spi_t* this,
		spi_clock_mode_t spi_clock_mode,
		spi_first_bit_t spi_first_bit,
		spi_prescaler_t spi_prescaler)
{
	assert( (spi_clock_mode == spi_clock_mode_idle_low_1_edge) ||
			(spi_clock_mode == spi_clock_mode_idle_high_1_edge) ||
			(spi_clock_mode == spi_clock_mode_idle_low_2_edge) ||
			(spi_clock_mode == spi_clock_mode_idle_high_2_edge) );

	assert( (spi_first_bit == spi_first_bit_msb) ||
			(spi_first_bit == spi_first_bit_lsb) );

	assert( (spi_prescaler == spi_prescaler_2 ) ||
			(spi_prescaler == spi_prescaler_4 ) ||
			(spi_prescaler == spi_prescaler_8 ) ||
			(spi_prescaler == spi_prescaler_16 ) ||
			(spi_prescaler == spi_prescaler_32 ) ||
			(spi_prescaler == spi_prescaler_64 ) ||
			(spi_prescaler == spi_prescaler_128 ) ||
			(spi_prescaler == spi_prescaler_256 ) );

	assert( IS_VALID_SPI_OBJECT_POINTER(this) );

	prv_rccInit(this);
	prv_gpioInit(this);
	prv_spiInit(this, spi_clock_mode, spi_first_bit, spi_prescaler);
}

uint8_t SpiDriver_BlockingTransfer(spi_t* this, uint8_t data)
{
	this->SPIx->DR = (uint16_t) data;
	volatile uint16_t i = 0;
	while ( ! (this->SPIx->SR & SPI_I2S_FLAG_RXNE) && (++i != 0) )
	{
	}
	uint8_t receivedData = (uint8_t)(this->SPIx->DR);
	return receivedData;
}

static void prv_rccInit(spi_t* this)
{
	if (this->SPIx == SPI1)	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	}
	else if (this->SPIx == SPI2)	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	}
	else if (this->SPIx == SPI3)	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	}
	else {
		debug_printf("Wrong spi object\n");
		debug_exit();
	}
}

static void prv_gpioInit(spi_t* this)
{
	GPIO_InitTypeDef gpioIn;
	gpioIn.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpioIn.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitTypeDef gpioOut;
	gpioOut.GPIO_Mode = GPIO_Mode_AF_PP;
	gpioOut.GPIO_Speed = GPIO_Speed_50MHz;

	if (this->SPIx == SPI1)	{
		gpioIn.GPIO_Pin = GPIO_Pin_6;
		GPIO_Init(GPIOA, &gpioIn);

		gpioOut.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
		GPIO_Init(GPIOA, &gpioOut);
	}
	else if (this->SPIx == SPI2)	{
		gpioIn.GPIO_Pin = GPIO_Pin_14;
		GPIO_Init(GPIOB, &gpioIn);

		gpioOut.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
		GPIO_Init(GPIOB, &gpioOut);
	}
	else if (this->SPIx == SPI3)	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
		gpioIn.GPIO_Pin = GPIO_Pin_4;
		GPIO_Init(GPIOB, &gpioIn);

		gpioOut.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5;
		GPIO_Init(GPIOB, &gpioOut);
	}
	else	{
		debug_printf("Wrong spi object\n");
		debug_exit();
	}
}

static void prv_spiInit(
		spi_t *this,
		spi_clock_mode_t spi_clock_mode,
		spi_first_bit_t spi_first_bit,
		spi_prescaler_t spi_prescaler)
{
	SPI_InitTypeDef spi;

	spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	spi.SPI_Mode = SPI_Mode_Master;
	spi.SPI_DataSize = SPI_DataSize_8b;

	if (spi_first_bit == spi_first_bit_msb)	{
		spi.SPI_FirstBit = SPI_FirstBit_MSB;
	}
	else	{
		spi.SPI_FirstBit = SPI_FirstBit_LSB;
	}

	switch (spi_clock_mode)
	{
	case spi_clock_mode_idle_low_1_edge:
		spi.SPI_CPOL = SPI_CPOL_Low;
		spi.SPI_CPHA = SPI_CPHA_1Edge;
		break;

	case spi_clock_mode_idle_high_1_edge:
		spi.SPI_CPOL = SPI_CPOL_High;
		spi.SPI_CPHA = SPI_CPHA_1Edge;
		break;

	case spi_clock_mode_idle_low_2_edge:
		spi.SPI_CPOL = SPI_CPOL_Low;
		spi.SPI_CPHA = SPI_CPHA_2Edge;
		break;

	case spi_clock_mode_idle_high_2_edge:
		spi.SPI_CPOL = SPI_CPOL_High;
		spi.SPI_CPHA = SPI_CPHA_2Edge;
		break;
	}

	switch (spi_prescaler)
	{
	case spi_prescaler_2:
		spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
		break;
	case spi_prescaler_4:
		spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
		break;
	case spi_prescaler_8:
		spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
		break;
	case spi_prescaler_16:
		spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
		break;
	case spi_prescaler_32:
		spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
		break;
	case spi_prescaler_64:
		spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
		break;
	case spi_prescaler_128:
		spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
		break;
	case spi_prescaler_256:
		spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
		break;
	}

	spi.SPI_NSS = SPI_NSS_Soft;
	spi.SPI_CRCPolynomial = 7;
	SPI_Init(this->SPIx, &spi);
	SPI_Cmd(this->SPIx, ENABLE);
}
