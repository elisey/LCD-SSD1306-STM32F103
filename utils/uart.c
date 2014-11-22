#include "uart.h"
#ifdef USE_FREERTOS
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#endif
typedef struct {
	uint8_t data[uartSIZE_OF_RING_BUFFER];
	unsigned int wrIdx;
	unsigned int rdIdx;
} sRingBuf_t;

#ifdef USE_FREERTOS
xSemaphoreHandle xRxSemaphore;
xSemaphoreHandle xTxMutex;
#endif
sRingBuf_t sTxRingBuf, sRxRingBuf;

void UART_Init(void)
{
	GPIO_InitTypeDef gpioStruct;
	USART_InitTypeDef usartStruct;

	USART_RCC_INIT;

	gpioStruct.GPIO_Pin = USART_Pin_Tx;
	gpioStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	gpioStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART_GPIO,&gpioStruct);

	//GPIO_RX
	gpioStruct.GPIO_Pin = USART_Pin_Rx;
	gpioStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpioStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART_GPIO,&gpioStruct);

	//USART_Settings
	usartStruct.USART_BaudRate = UART_BOUD_RATE;
	usartStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usartStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usartStruct.USART_Parity = USART_Parity_No;
	usartStruct.USART_StopBits = USART_StopBits_1;
	usartStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(USARTx, &usartStruct);

	USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
	USART_Cmd(USARTx, ENABLE);

#ifdef USE_FREERTOS
	xRxSemaphore = xSemaphoreCreateCounting(10,0);
	xTxMutex = xSemaphoreCreateMutex();
	NVIC_SetPriority(USARTx_IRQn, 12);	//TODO приоритет для работы совместно с FreeRTOS. Проверить
#endif
	NVIC_EnableIRQ(USARTx_IRQn);
}

void UART_SendChar(uint8_t data)
{
#ifdef USE_FREERTOS
	xSemaphoreTake(xTxMutex, portMAX_DELAY);
#else
	__disable_irq();
#endif

	sTxRingBuf.data[sTxRingBuf.wrIdx++] = data;
	if (sTxRingBuf.wrIdx >= uartSIZE_OF_RING_BUFFER)	{
		sTxRingBuf.wrIdx = 0;
	}
	USART_ITConfig(USARTx, USART_IT_TXE, ENABLE);

#ifdef USE_FREERTOS
	xSemaphoreGive(xTxMutex);
#else
	__enable_irq();
#endif
}

void UART_SendString(const char *str)
{
	int i = 0;
	while (str[i] != 0)
	{
		UART_SendChar(str[i]);
		i++;
	}
}

int UART_GetChar()
{
	uint8_t data;
	if (sRxRingBuf.wrIdx != sRxRingBuf.rdIdx)	{
		data = sRxRingBuf.data[sRxRingBuf.rdIdx++];
		if (sRxRingBuf.rdIdx >= uartSIZE_OF_RING_BUFFER)	{
			sRxRingBuf.rdIdx = 0;
		}
		return (int)data;
	}
	else	{
		return (UART_NO_DATA);
	}
}

#ifdef USE_FREERTOS
int UART_GetCharBlocking()
{
	xSemaphoreTake(xRxSemaphore, portMAX_DELAY);
	return UART_GetChar();
}
#endif

void USARTx_IRQHandler (void)
{
	if (USART_GetITStatus(USARTx,USART_IT_RXNE) != RESET)	{
		USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
		sRxRingBuf.data[sRxRingBuf.wrIdx++] = USART_ReceiveData(USARTx);
		if (sRxRingBuf.wrIdx >= uartSIZE_OF_RING_BUFFER)	{
			sRxRingBuf.wrIdx = 0;
		}
#ifdef USE_FREERTOS
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(xRxSemaphore, &xHigherPriorityTaskWoken);
		if( xHigherPriorityTaskWoken != pdFALSE )	{
			portYIELD();
		}
#endif
	}
	else if (USART_GetITStatus(USARTx,USART_IT_TXE) != RESET)	{
		if (sTxRingBuf.wrIdx != sTxRingBuf.rdIdx)	{
			USART_SendData(USARTx, sTxRingBuf.data[sTxRingBuf.rdIdx++]);
			if (sTxRingBuf.rdIdx >= uartSIZE_OF_RING_BUFFER)	{
				sTxRingBuf.rdIdx = 0;
			}
			if (sTxRingBuf.wrIdx == sTxRingBuf.rdIdx)	{
				USART_ITConfig(USARTx, USART_IT_TXE, DISABLE);
			}
		}
	}
}
