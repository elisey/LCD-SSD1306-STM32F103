#include "ssd1306_HAL.h"

#include "spi.h"
#include "stm32f10x.h"

static spi_t *const ptrSpiDriver = &spi1;
static const uint32_t CS_RCC = RCC_APB2Periph_GPIOA;
static  GPIO_TypeDef* const CS_GPIO = GPIOA;
static const uint16_t CS_Pin = GPIO_Pin_2;

static const uint32_t DC_RCC = RCC_APB2Periph_GPIOA;
static  GPIO_TypeDef* const DC_GPIO = GPIOA;
static const uint16_t DC_Pin = GPIO_Pin_3;

static const uint32_t RESET_RCC = RCC_APB2Periph_GPIOA;
static  GPIO_TypeDef* const RESET_GPIO = GPIOA;
static const uint16_t RESET_Pin = GPIO_Pin_4;

void Ssd1306_HAL_Init()
{
	RCC_APB2PeriphClockCmd(CS_RCC | DC_RCC | RESET_RCC, ENABLE);

	GPIO_InitTypeDef gpio;

	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;

	gpio.GPIO_Pin = CS_Pin;
	GPIO_Init(CS_GPIO, &gpio);

	gpio.GPIO_Pin = DC_Pin;
	GPIO_Init(DC_GPIO, &gpio);

	gpio.GPIO_Pin = RESET_Pin;
	GPIO_Init(RESET_GPIO, &gpio);

	Ssd1306_HAL_ChipDeselect();
	Ssd1306_HAL_ComandSelect();
	Ssd1306_HAL_ResetDisnable();

	SpiDriver_Init(ptrSpiDriver, spi_clock_mode_idle_low_1_edge, spi_first_bit_msb, spi_prescaler_2);
}

uint8_t Ssd1306_HAL_SpiTransfer(uint8_t data)
{
	return SpiDriver_BlockingTransfer(ptrSpiDriver, data);
}

void Ssd1306_HAL_ChipSelect()
{
	GPIO_WriteBit(CS_GPIO, CS_Pin, Bit_RESET);
}

void Ssd1306_HAL_ChipDeselect()
{
	GPIO_WriteBit(CS_GPIO, CS_Pin, Bit_SET);
}

void Ssd1306_HAL_DataSelect()
{
	GPIO_WriteBit(DC_GPIO, DC_Pin, Bit_SET);
}

void Ssd1306_HAL_ComandSelect()
{
	GPIO_WriteBit(DC_GPIO, DC_Pin, Bit_RESET);
}

void Ssd1306_HAL_ResetEnable()
{
	GPIO_WriteBit(RESET_GPIO, RESET_Pin, Bit_RESET);
}

void Ssd1306_HAL_ResetDisnable()
{
	GPIO_WriteBit(RESET_GPIO, RESET_Pin, Bit_SET);
}
