#include "ssd1306_driver.h"

#include "spi.h"
#include "stm32f10x.h"
#include "cmsis/core_cm3.h"	// NVIC_SetPriority, NVIC_EnableIRQ

#include "FreeRTOS.h"
#include "semphr.h"			// mutex
#include "task.h"			// vTaskDelayUntil

#include <string.h>			// for memset
#include "debug.h"

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

static const uint8_t SSD1306_SETCONTRAST = 0x81;
static const uint8_t SSD1306_DISPLAYALLON_RESUME = 0xA4;
static const uint8_t SSD1306_DISPLAYALLON = 0xA5;
static const uint8_t SSD1306_NORMALDISPLAY = 0xA6;
static const uint8_t SSD1306_INVERTDISPLAY = 0xA7;
static const uint8_t SSD1306_DISPLAYOFF = 0xAE;
static const uint8_t SSD1306_DISPLAYON = 0xAF;
static const uint8_t SSD1306_SETDISPLAYOFFSET = 0xD3;
static const uint8_t SSD1306_SETCOMPINS = 0xDA;
static const uint8_t SSD1306_SETVCOMDETECT = 0xDB;
static const uint8_t SSD1306_SETDISPLAYCLOCKDIV = 0xD5;
static const uint8_t SSD1306_SETPRECHARGE = 0xD9;
static const uint8_t SSD1306_SETMULTIPLEX = 0xA8;
static const uint8_t SSD1306_SETLOWCOLUMN = 0x00;
static const uint8_t SSD1306_SETHIGHCOLUMN = 0x10;
static const uint8_t SSD1306_SETSTARTLINE = 0x40;
static const uint8_t SSD1306_MEMORYMODE = 0x20;
static const uint8_t SSD1306_COMSCANINC = 0xC0;
static const uint8_t SSD1306_COMSCANDEC = 0xC8;
static const uint8_t SSD1306_SEGREMAP = 0xA0;
static const uint8_t SSD1306_CHARGEPUMP = 0x8D;
static const uint8_t SSD1306_EXTERNALVCC = 0x1;
static const uint8_t SSD1306_SWITCHCAPVCC = 0x2;

#define SSD1306_LCDWIDTH		128
#define SSD1306_LCDHEIGHT		64

#define SSD1306_BUFFER_SIZE		(SSD1306_LCDWIDTH * SSD1306_LCDHEIGHT / 8)

static DMA_InitTypeDef dma;

uint8_t buffer[SSD1306_BUFFER_SIZE];
uint8_t cmdBuffer[] = { 0x00 | 0x0,		// SSD1306_SETLOWCOLUMN | 0x0
						0x10 | 0x0,		// SSD1306_SETHIGHCOLUMN | 0x0
						0x40 | 0x0 };	// SSD1306_SETSTARTLINE | 0x0

SemaphoreHandle_t bufferMutex;

static void prv_gpioInit();
static void prv_spiInit();
static void prv_dmaInit();
static void prv_lcdControllerInit();
static void prv_resetLcdController();
static void prv_sendCmd(uint8_t cmd);
static void prv_startCmdOutput();
static void prv_startDataOutput();
static void prv_stopOutput();

static uint8_t prv_spiTransfer(uint8_t data);
static void prv_chipSelect();
static void prv_chipDeselect();
static void prv_dataSelect();
static void prv_comandSelect();
static void prv_resetEnable();
static void prv_resetDisnable();


void Ssd1306_Task(void *pvParameters)
{
	bufferMutex = xSemaphoreCreateMutex();

	prv_gpioInit();
	prv_spiInit();
	prv_dmaInit();
	prv_lcdControllerInit();

	portTickType lastWakeTime;
	lastWakeTime = xTaskGetTickCount();

	while(1)	{
		Ssd1306_HAL_Lock();
		prv_startCmdOutput();

		vTaskDelayUntil( &lastWakeTime, ssd1306REFRESH_RATE_MS / portTICK_RATE_MS );
	}
}

void Ssd1306_HAL_Lock()
{
	xSemaphoreTake(bufferMutex, portMAX_DELAY);
}
void Ssd1306_HAL_Unlock()
{
	xSemaphoreGive(bufferMutex);
}

void Ssd1306_DrawPixel(uint8_t x, uint8_t y)
{
	assert(x < SSD1306_LCDWIDTH);
	assert(y < SSD1306_LCDHEIGHT);

	buffer[x + (y / 8) * SSD1306_LCDWIDTH] |= (1 << y % 8);
}

void Ssd1306_ClearPixel(uint8_t x, uint8_t y)
{
	assert(x < SSD1306_LCDWIDTH);
	assert(y < SSD1306_LCDHEIGHT);

	buffer[x + (y / 8) * SSD1306_LCDWIDTH] &= ~(1 << y % 8);
}

uint8_t Ssd1306_GetPixel(uint8_t x, uint8_t y)
{
	assert(x < SSD1306_LCDWIDTH);
	assert(y < SSD1306_LCDHEIGHT);
	return buffer[x + (y / 8) * SSD1306_LCDWIDTH] & (1 << y % 8) ? 1 : 0;
}

void Ssd1306_ClearScreen(void)
{
	memset((void*) buffer, 0, 1024);
}

static void prv_gpioInit()
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

	prv_chipDeselect();
	prv_comandSelect();
	prv_resetDisnable();
}

static void prv_spiInit()
{
	SpiDriver_Init(ptrSpiDriver, spi_clock_mode_idle_low_1_edge, spi_first_bit_msb, spi_prescaler_8);
}

static void prv_dmaInit()
{
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	dma.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;
	dma.DMA_MemoryBaseAddr = (uint32_t)NULL;
	dma.DMA_DIR = DMA_DIR_PeripheralDST;
	dma.DMA_BufferSize = 0;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_Mode = DMA_Mode_Normal;
	dma.DMA_Priority = DMA_Priority_Medium;
	dma.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel3, &dma);
	DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);
	NVIC_SetPriority(DMA1_Channel3_IRQn, 13);
	NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

static void prv_lcdControllerInit()
{
	uint8_t vccstate = 0;
	 prv_resetLcdController();

	// Initialisation sequence
	prv_sendCmd(SSD1306_DISPLAYOFF); // 0xAE
	prv_sendCmd(SSD1306_SETLOWCOLUMN | 0x0); // low col = 0
	prv_sendCmd(SSD1306_SETHIGHCOLUMN | 0x0); // hi col = 0
	prv_sendCmd(SSD1306_SETSTARTLINE | 0x0); // line #0
	prv_sendCmd(SSD1306_SETCONTRAST); // 0x81
	if (vccstate == SSD1306_EXTERNALVCC) {
		prv_sendCmd(0x9F);
	} else {
		prv_sendCmd(0xCF);
	}
	prv_sendCmd(0xa1); // setment remap 95 to 0 (?)
	prv_sendCmd(SSD1306_NORMALDISPLAY); // 0xA6
	prv_sendCmd(SSD1306_DISPLAYALLON_RESUME); // 0xA4
	prv_sendCmd(SSD1306_SETMULTIPLEX); // 0xA8
	prv_sendCmd(0x3F); // 0x3F 1/64 duty
	prv_sendCmd(SSD1306_SETDISPLAYOFFSET); // 0xD3
	prv_sendCmd(0x0); // no offset
	prv_sendCmd(SSD1306_SETDISPLAYCLOCKDIV); // 0xD5
	//prv_sendCmd(0x80);                                // the suggested ratio 0x80
	prv_sendCmd(0xF0); //MAX clock rate
	prv_sendCmd(SSD1306_SETPRECHARGE); // 0xd9
	if (vccstate == SSD1306_EXTERNALVCC) {
		prv_sendCmd(0x22);
	} else {
		prv_sendCmd(0xF1);
	}
	prv_sendCmd(SSD1306_SETCOMPINS); // 0xDA
	prv_sendCmd(0x12); // disable COM left/right remap
	prv_sendCmd(SSD1306_SETVCOMDETECT); // 0xDB
	prv_sendCmd(0x40); // 0x20 is default?
	prv_sendCmd(SSD1306_MEMORYMODE); // 0x20
	prv_sendCmd(0x00); // 0x0 act like ks0108
	prv_sendCmd(SSD1306_SEGREMAP | 0x1);
	prv_sendCmd(SSD1306_COMSCANDEC);
	prv_sendCmd(SSD1306_CHARGEPUMP); //0x8D
	if (vccstate == SSD1306_EXTERNALVCC) {
		prv_sendCmd(0x10);
	} else {
		prv_sendCmd(0x14);
	}
	// Enabled the OLED panel
	prv_sendCmd(SSD1306_DISPLAYON);
}

static void prv_resetLcdController()
{
	prv_resetDisnable();
	vTaskDelay(1 / portTICK_RATE_MS);
	prv_resetEnable();
	vTaskDelay(5 / portTICK_RATE_MS);
	prv_resetDisnable();
}

static void prv_sendCmd(uint8_t cmd)
{
	prv_chipSelect();
	prv_comandSelect();
	prv_spiTransfer(cmd);
	prv_chipDeselect();
}

enum	{
	out_none = 0,
	out_cmd,
	out_data
} currentOutType = out_none;

void DMA1_Channel3_IRQHandler()
{
	if (DMA_GetITStatus(DMA1_IT_TC3) == SET)	{
		DMA_ClearITPendingBit(DMA1_IT_TC3);

		switch (currentOutType) {
			case out_none:
				//TODO ASSERT
				break;

			case out_cmd:
				prv_startDataOutput();
				break;

			case out_data:
				prv_stopOutput();
				break;

			default:
				//TODO ASSERT
				break;
		}
	}
}

static void prv_startCmdOutput()
{
	if (currentOutType != out_none)	{
		//TODO ASSERT
		while(1);
	}

	currentOutType = out_cmd;

	prv_chipSelect();
	prv_comandSelect();

	DMA_Cmd(DMA1_Channel3, DISABLE);
	dma.DMA_MemoryBaseAddr = (uint32_t)cmdBuffer;
	dma.DMA_BufferSize = sizeof(cmdBuffer);
	DMA_Init(DMA1_Channel3, &dma);
	DMA_Cmd(DMA1_Channel3, ENABLE);
}

static void prv_startDataOutput()
{
	currentOutType = out_data;

	prv_chipDeselect();
	prv_dataSelect();
	prv_chipSelect();

	DMA_Cmd(DMA1_Channel3, DISABLE);
	dma.DMA_MemoryBaseAddr = (uint32_t)buffer;
	dma.DMA_BufferSize = sizeof(buffer);
	DMA_Init(DMA1_Channel3, &dma);
	DMA_Cmd(DMA1_Channel3, ENABLE);
}

static void prv_stopOutput()
{
	currentOutType = out_none;
	prv_chipDeselect();
	xSemaphoreGiveFromISR(bufferMutex, NULL);
}

static uint8_t prv_spiTransfer(uint8_t data)
{
	return SpiDriver_BlockingTransfer(ptrSpiDriver, data);
}

static void prv_chipSelect()
{
	GPIO_WriteBit(CS_GPIO, CS_Pin, Bit_RESET);
}

static void prv_chipDeselect()
{
	GPIO_WriteBit(CS_GPIO, CS_Pin, Bit_SET);
}

static void prv_dataSelect()
{
	GPIO_WriteBit(DC_GPIO, DC_Pin, Bit_SET);
}

static void prv_comandSelect()
{
	GPIO_WriteBit(DC_GPIO, DC_Pin, Bit_RESET);
}

static void prv_resetEnable()
{
	GPIO_WriteBit(RESET_GPIO, RESET_Pin, Bit_RESET);
}

static void prv_resetDisnable()
{
	GPIO_WriteBit(RESET_GPIO, RESET_Pin, Bit_SET);
}
