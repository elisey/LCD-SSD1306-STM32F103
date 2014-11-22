#include "ssd1306_driver.h"
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

#include "uart.h"

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

#include "font_anonymous_pro.h"
#include "font_trebuchet_9.h"
#include "font_pixeldust_24.h"
#include "font_bitlow_28.h"
void lcdProcess(void *param)
{
	while(1)
	{
		uint16_t i;
		for (i = 0; i < 127; ++i) {
			Ssd1306_HAL_Lock();
			Ssd1306_ClearScreen();
			//LCD_DrawStraightLine(i, 0, i, 63);
			fontsDrawString(0,0, &trebuchetMS9ptFontInfo, "Test msg!");
			fontsDrawString(10,30, &bitLow28ptFontInfo, "045");
			//fontsDrawString(0,40, &trebuchetMS9ptFontInfo, "1234567890 !@#$%^&");
			Ssd1306_HAL_Unlock();
			vTaskDelay(15/portTICK_RATE_MS);
		}
	}

}



int main(void)
{

	UART_Init();
	xTaskCreate(
			Ssd1306_Task,
			"RadioHandler",
			200,
			NULL,
			tskIDLE_PRIORITY + 2,
			NULL);

	xTaskCreate(
			lcdProcess,
			"RadioHandler",
			200,
			NULL,
			tskIDLE_PRIORITY + 1,
			NULL);

	//Ssd1306_HAL_Init();



	vTaskStartScheduler();

    while(1)
    {

    }
}






void delay()
{
	volatile int i;
	for (i = 0; i < 50000; ++i);
}
