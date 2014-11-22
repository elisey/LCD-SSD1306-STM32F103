#include "ssd1306_HAL.h"

#include "spi.h"
#include "stm32f10x.h"
#include "led.h"
#include "FreeRTOS.h"
#include "semphr.h"


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

#define Delay_TIMx						TIM2
#define Delay_RCC_APB					RCC_APB1Periph_TIM2
#define Delay_DBGMCU_CR_DBG_TIMx_STOP	DBGMCU_CR_DBG_TIM2_STOP


#define SSD1306_BUFFER_SIZE					(SSD1306_LCDWIDTH * SSD1306_LCDHEIGHT / 8)
DMA_InitTypeDef dma;
uint8_t buffer[SSD1306_BUFFER_SIZE];
uint8_t cmdBuffer[3] = { 0x00 | 0x0, 0x10 | 0x0, 0x40 | 0x0 };

Led_t blueLed;

SemaphoreHandle_t bufferMutex;

static void prv_timerInit();
static void prv_dmaInit();
static void prv_startDataOutput();
static void prv_process();

void Ssd1306_HAL_Init(void *pvParameters)
{
	Led_Init(&blueLed, GPIOC, GPIO_Pin_8);
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

	prv_dmaInit();

	bufferMutex = xSemaphoreCreateMutex();



	//prv_timerInit();
}

void Ssd1306_HAL_Process(void *param)
{
	while(1)
	{
		Ssd1306_HAL_Lock();
		prv_startDataOutput();

		vTaskDelay(10 / portTICK_RATE_MS);
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

void Ssd1306_HAL_Start()
{
	NVIC_EnableIRQ(TIM2_IRQn);
	TIM_Cmd( Delay_TIMx, ENABLE );
}

void Ssd1306_HAL_Stop()
{
	NVIC_DisableIRQ(TIM2_IRQn);
	TIM_Cmd( Delay_TIMx, DISABLE );
}

static void prv_timerInit()
{
	RCC_APB1PeriphClockCmd( Delay_RCC_APB, ENABLE );

	RCC_ClocksTypeDef clock;
	RCC_GetClocksFreq(&clock);

	TIM_TimeBaseInitTypeDef timerStruct;
	timerStruct.TIM_ClockDivision = 0;
	timerStruct.TIM_CounterMode = TIM_CounterMode_Up;
	timerStruct.TIM_Period = 20;
	timerStruct.TIM_Prescaler = ((clock.PCLK1_Frequency / 1000ul) - 1);
	TIM_TimeBaseInit( Delay_TIMx, &timerStruct );

	TIM_ITConfig(Delay_TIMx, TIM_IT_Update, ENABLE);
	//NVIC_SetPriority(TIM2_IRQn, 1);
	//TIM_Cmd( Delay_TIMx, ENABLE );
	//NVIC_EnableIRQ(TIM2_IRQn);
	DBGMCU->CR |= Delay_DBGMCU_CR_DBG_TIMx_STOP;
}

static void prv_dmaInit()
{
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	dma.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;
	//dma.DMA_MemoryBaseAddr = buffer;
	dma.DMA_DIR = DMA_DIR_PeripheralDST;
	//dma.DMA_BufferSize = SSD1306_BUFFER_SIZE;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_Mode = DMA_Mode_Normal;
	dma.DMA_Priority = DMA_Priority_Medium;
	dma.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel3, &dma);
	DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);
	NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

typedef enum	{
	state_wait = 0,
	state_cmd,
	state_data
} state_t;

static state_t state = state_wait;

static void prv_startDataOutput()
{
	if (state != state_wait)	{
		return;
	}

	state = state_cmd;

	Ssd1306_HAL_ChipSelect();
	Ssd1306_HAL_ComandSelect();

	DMA_Cmd(DMA1_Channel3, DISABLE);
	dma.DMA_MemoryBaseAddr = (uint32_t)cmdBuffer;
	dma.DMA_BufferSize = 3;
	DMA_Init(DMA1_Channel3, &dma);
	DMA_Cmd(DMA1_Channel3, ENABLE);
}



static void prv_process()
{
	Led_SetState(&blueLed, led_on);


	switch (state) {
		case state_wait:

			break;

		case state_cmd:
			state = state_data;

			Ssd1306_HAL_ChipDeselect();
			Ssd1306_HAL_DataSelect();
			Ssd1306_HAL_ChipSelect();

			DMA_Cmd(DMA1_Channel3, DISABLE);
			dma.DMA_MemoryBaseAddr = (uint32_t)buffer;
			dma.DMA_BufferSize = SSD1306_BUFFER_SIZE;
			DMA_Init(DMA1_Channel3, &dma);
			DMA_Cmd(DMA1_Channel3, ENABLE);
			break;

		case state_data:
			state = state_wait;
			Ssd1306_HAL_ChipDeselect();
			xSemaphoreGiveFromISR(bufferMutex, NULL);
			break;

		default:
			break;
	}
	Led_SetState(&blueLed, led_off);
}

/*void TIM2_IRQHandler()
{
	if (TIM_GetITStatus(Delay_TIMx, TIM_IT_Update) == SET)	{
		TIM_ClearITPendingBit(Delay_TIMx, TIM_IT_Update);
		prv_process();
	}
}*/

void DMA1_Channel3_IRQHandler()
{
	if (DMA_GetITStatus(DMA1_IT_TC3) == SET)	{
		DMA_ClearITPendingBit(DMA1_IT_TC3);
		prv_process();
	}
}


