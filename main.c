#include "ssd1306.h"
#include "ssd1306_HAL.h"
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

void lcdProcess(void *param)
{
	while(1)
	{
		int i;
		for (i = 0; i < 127; ++i) {
			Ssd1306_HAL_Lock();
			Ssd1306_ClearScreen();
			LCD_DrawStraightLine(i, 0, i, 63);
			Ssd1306_HAL_Unlock();
			vTaskDelay(15/portTICK_RATE_MS);
		}
	}

}

int main(void)
{

	xTaskCreate(
			Ssd1306_HAL_Process,
			"RadioHandler",
			200,
			NULL,
			tskIDLE_PRIORITY + 1,
			NULL);

	xTaskCreate(
			lcdProcess,
			"RadioHandler",
			200,
			NULL,
			tskIDLE_PRIORITY + 1,
			NULL);

	//Ssd1306_HAL_Init();
	Ssd1306_Init(0);


	vTaskStartScheduler();

    while(1)
    {

    }
}




void LCD_DrawStraightLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    if (x0 == x1)   {
        if (y1>y0)   {
            for ( ; y0<=y1; y0++)    {
            	Ssd1306_DrawPixel(x0,y0);
            }
        }
        else    {
            for ( ; y1<=y0; y1++)    {
            	Ssd1306_DrawPixel(x0,y1);
            }
        }
    }
    else if (y0 == y1)  {
        if (x1>x0)   {
            for ( ; x0<=x1; x0++)    {
            	Ssd1306_DrawPixel(x0,y0);
            }
        }
        else    {
            for ( ; x1<=x0; x1++)    {
            	Ssd1306_DrawPixel(x1,y0);
            }
        }
    }
}

void delay()
{
	volatile int i;
	for (i = 0; i < 50000; ++i);
}
