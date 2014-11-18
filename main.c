#include "ssd1306.h"
int main(void)
{
	Ssd1306_Init(0);
	//Ssd1306_Refresh();

	int i,j;
	for (j = 0; j < 60; ++j) {
		for (i = 0; i < 40; ++i) {
			Ssd1306_DrawPixel(45 + i, 1 + j);
		}
	}
	Ssd1306_Refresh();
    while(1)
    {
    	Ssd1306_ClearScreen();

    	int i, j;
    	for (i = 0; i < 100000; ++i);

    	for (j = 0; j < 60; ++j) {
    		for (i = 0; i < 40; ++i) {
    			Ssd1306_DrawPixel(45 + i, 1 + j);
    		}
    	}

    	for (i = 0; i < 1000000; ++i);
    }
}
