/**************************************************************************/
/*! 
    @file     ssd1306.c
    @author   K. Townsend (microBuilder.eu)

    @section DESCRIPTION

    Driver for 128x64 OLED display based on the SSD1306 controller.

    This driver is based on the SSD1306 Library from Limor Fried
    (Adafruit Industries) at: https://github.com/adafruit/SSD1306  
    
    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2010, microBuilder SARL
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#include <string.h>

#include "ssd1306.h"
#include "ssd1306_HAL.h"

#define DELAY(mS)

uint8_t buffer[SSD1306_LCDWIDTH * SSD1306_LCDHEIGHT / 8];

static void prv_sendCmd(uint8_t cmd)
{
	Ssd1306_HAL_ChipDeselect();		//FIXME может не обязательно
	Ssd1306_HAL_ComandSelect();
	Ssd1306_HAL_ChipSelect();
	Ssd1306_HAL_SpiTransfer(cmd);
	Ssd1306_HAL_ChipDeselect();
}

static void prv_sendData(uint8_t data)
{
	Ssd1306_HAL_ChipDeselect();		//FIXME может не обязательно
	Ssd1306_HAL_DataSelect();
	Ssd1306_HAL_ChipSelect();
	Ssd1306_HAL_SpiTransfer(data);
	Ssd1306_HAL_ChipDeselect();
}


/**************************************************************************/
/*!
    @brief  Draws a single graphic character using the supplied font
*/
/**************************************************************************/
/*static void Ssd1306_DrawChar(uint16_t x, uint16_t y, uint8_t c, struct FONT_DEF font)
{
  uint8_t col, column[font.u8Width];

  // Check if the requested character is available
  if ((c >= font.u8FirstChar) && (c <= font.u8LastChar))
  {
    // Retrieve appropriate columns from font data
    for (col = 0; col < font.u8Width; col++)
    {
      column[col] = font.au8FontTable[((c - 32) * font.u8Width) + col];    // Get first column of appropriate character
    }
  }
  else
  {    
    // Requested character is not available in this font ... send a space instead
    for (col = 0; col < font.u8Width; col++)
    {
      column[col] = 0xFF;    // Send solid space
    }
  }

  // Render each column
  uint16_t xoffset, yoffset;
  for (xoffset = 0; xoffset < font.u8Width; xoffset++)
  {
    for (yoffset = 0; yoffset < (font.u8Height + 1); yoffset++)
    {
      uint8_t bit = 0x00;
      bit = (column[xoffset] << (8 - (yoffset + 1)));     // Shift current row bit left
      bit = (bit >> 7);                     // Shift current row but right (results in 0x01 for black, and 0x00 for white)
      if (bit)
      {
        Ssd1306_DrawPixel(x + xoffset, y + yoffset);
      }
    }
  }
}*/

/**************************************************************************/
/* Public Methods                                                         */
/**************************************************************************/

/**************************************************************************/
/*! 
    @brief Initialises the SSD1306 LCD display
*/
/**************************************************************************/
void Ssd1306_Init(uint8_t vccstate)
{
	Ssd1306_HAL_Init();
	Ssd1306_HAL_ResetDisnable();
	DELAY(1);
	Ssd1306_HAL_ResetEnable();
	DELAY(1);
	Ssd1306_HAL_ResetDisnable();

  // Initialisation sequence
  prv_sendCmd(SSD1306_DISPLAYOFF);                    // 0xAE
  prv_sendCmd(SSD1306_SETLOWCOLUMN | 0x0);            // low col = 0
  prv_sendCmd(SSD1306_SETHIGHCOLUMN | 0x0);           // hi col = 0
  prv_sendCmd(SSD1306_SETSTARTLINE | 0x0);            // line #0
  prv_sendCmd(SSD1306_SETCONTRAST);                   // 0x81
  if (vccstate == SSD1306_EXTERNALVCC) 
    { prv_sendCmd(0x9F); }
  else 
    { prv_sendCmd(0xCF); }
  prv_sendCmd(0xa1);                                  // setment remap 95 to 0 (?)
  prv_sendCmd(SSD1306_NORMALDISPLAY);                 // 0xA6
  prv_sendCmd(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
  prv_sendCmd(SSD1306_SETMULTIPLEX);                  // 0xA8
  prv_sendCmd(0x3F);                                  // 0x3F 1/64 duty
  //prv_sendCmd(16);
  prv_sendCmd(SSD1306_SETDISPLAYOFFSET);              // 0xD3
  prv_sendCmd(0x0);                                   // no offset
  prv_sendCmd(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
  //prv_sendCmd(0x80);                                // the suggested ratio 0x80
  prv_sendCmd(0xF0);									//MAX clock rate
  prv_sendCmd(SSD1306_SETPRECHARGE);                  // 0xd9
  if (vccstate == SSD1306_EXTERNALVCC) 
    { prv_sendCmd(0x22); }
  else 
    { prv_sendCmd(0xF1); }
  prv_sendCmd(SSD1306_SETCOMPINS);                    // 0xDA
  prv_sendCmd(0x12);                                  // disable COM left/right remap
  prv_sendCmd(SSD1306_SETVCOMDETECT);                 // 0xDB
  prv_sendCmd(0x40);                                  // 0x20 is default?
  prv_sendCmd(SSD1306_MEMORYMODE);                    // 0x20
  prv_sendCmd(0x00);                                  // 0x0 act like ks0108
  prv_sendCmd(SSD1306_SEGREMAP | 0x1);
  prv_sendCmd(SSD1306_COMSCANDEC);
  prv_sendCmd(SSD1306_CHARGEPUMP);                    //0x8D
  if (vccstate == SSD1306_EXTERNALVCC) 
    { prv_sendCmd(0x10); }
  else 
    { prv_sendCmd(0x14); }

  // Enabled the OLED panel
  prv_sendCmd(SSD1306_DISPLAYON);
}

/**************************************************************************/
/*! 
    @brief Draws a single pixel in image buffer

    @param[in]  x
                The x position (0..127)
    @param[in]  y
                The y position (0..63)
*/
/**************************************************************************/
void Ssd1306_DrawPixel(uint8_t x, uint8_t y)
{
  if ((x >= SSD1306_LCDWIDTH) || (y >= SSD1306_LCDHEIGHT))
    return;

  buffer[x+ (y/8)*SSD1306_LCDWIDTH] |= (1 << y%8);
}

/**************************************************************************/
/*! 
    @brief Clears a single pixel in image buffer

    @param[in]  x
                The x position (0..127)
    @param[in]  y
                The y position (0..63)
*/
/**************************************************************************/
void Ssd1306_ClearPixel(uint8_t x, uint8_t y)
{
  if ((x >= SSD1306_LCDWIDTH) || (y >= SSD1306_LCDHEIGHT))
    return;

  buffer[x+ (y/8)*SSD1306_LCDWIDTH] &= ~(1 << y%8); 
}

/**************************************************************************/
/*! 
    @brief Gets the value (1 or 0) of the specified pixel from the buffer

    @param[in]  x
                The x position (0..127)
    @param[in]  y
                The y position (0..63)

    @return     1 if the pixel is enabled, 0 if disabled
*/
/**************************************************************************/
uint8_t Ssd1306_GetPixel(uint8_t x, uint8_t y)
{
  if ((x >= SSD1306_LCDWIDTH) || (y >=SSD1306_LCDHEIGHT)) return 0;
  return buffer[x+ (y/8)*SSD1306_LCDWIDTH] & (1 << y%8) ? 1 : 0;
}

/**************************************************************************/
/*! 
    @brief Clears the screen
*/
/**************************************************************************/
void Ssd1306_ClearScreen(void)
{
  memset(buffer, 0, 1024);
  Ssd1306_Refresh();
}

/**************************************************************************/
/*! 
    @brief Renders the contents of the pixel buffer on the LCD
*/
/**************************************************************************/

#include "stm32f10x.h"
void Ssd1306_Refresh(void)
{
	prv_sendCmd(SSD1306_SETLOWCOLUMN | 0x0);  // low col = 0
	prv_sendCmd(SSD1306_SETHIGHCOLUMN | 0x0);  // hi col = 0
	prv_sendCmd(SSD1306_SETSTARTLINE | 0x0); // line #0

	Ssd1306_HAL_ChipDeselect();		//FIXME может не обязательно
	Ssd1306_HAL_DataSelect();
	Ssd1306_HAL_ChipSelect();

	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_InitTypeDef dma;

	dma.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;
	dma.DMA_MemoryBaseAddr = buffer;
	dma.DMA_DIR = DMA_DIR_PeripheralDST;
	dma.DMA_BufferSize = 1024;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_Mode = DMA_Mode_Circular;
	dma.DMA_Priority = DMA_Priority_Medium;
	dma.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel3, &dma);
	DMA_Cmd(DMA1_Channel3, ENABLE);

	int i;
	for (i = 0; i < 1000000; ++i) {

	}
	//Ssd1306_HAL_ChipDeselect();


/*  uint16_t i;
  for (i=0; i<1024; i++) 
  {
	  prv_sendData(buffer[i]);
  }*/
}

/**************************************************************************/
/*!
    @brief  Draws a string using the supplied font data.

    @param[in]  x
                Starting x co-ordinate
    @param[in]  y
                Starting y co-ordinate
    @param[in]  text
                The string to render
    @param[in]  font
                Pointer to the FONT_DEF to use when drawing the string

    @section Example

    @code 

    #include "drivers/lcd/bitmap/ssd1306/ssd1306.h"
    #include "drivers/lcd/smallfonts.h"
    
    // Configure the pins and initialise the LCD screen
    Ssd1306_Init();

    // Render some text on the screen
    Ssd1306_DrawString(1, 10, "5x8 System", Font_System5x8);
    Ssd1306_DrawString(1, 20, "7x8 System", Font_System7x8);

    // Refresh the screen to see the results
    Ssd1306_Refresh();

    @endcode
*/
/**************************************************************************/
/*void Ssd1306_DrawString(uint16_t x, uint16_t y, char* text, struct FONT_DEF font)
{
  uint8_t l;
  for (l = 0; l < strlen(text); l++)
  {
    Ssd1306_DrawChar(x + (l * (font.u8Width + 1)), y, text[l], font);
  }
}*/

/**************************************************************************/
/*!
    @brief  Shifts the contents of the frame buffer up the specified
            number of pixels

    @param[in]  height
                The number of pixels to shift the frame buffer up, leaving
                a blank space at the bottom of the frame buffer x pixels
                high

    @section Example

    @code 

    #include "drivers/lcd/bitmap/ssd1306/ssd1306.h"
    #include "drivers/lcd/smallfonts.h"
    
    // Configure the pins and initialise the LCD screen
    Ssd1306_Init();

    // Enable the backlight
    Ssd1306_BLEnable();

    // Continually write some text, scrolling upward one line each time
    while (1)
    {
      // Shift the buffer up 8 pixels (adjust for font-height)
      Ssd1306_ShiftFrameBuffer(8);
      // Render some text on the screen with different fonts
      Ssd1306_DrawString(1, 56, "INSERT TEXT HERE", Font_System5x8);
      // Refresh the screen to see the results
      Ssd1306_Refresh();
      // Wait a bit before writing the next line
      systickDelay(1000);
    }

    @endcode
*/
/**************************************************************************/
void Ssd1306_ShiftFrameBuffer( uint8_t height )
{
  if (height == 0) return;
  if (height >= SSD1306_LCDHEIGHT)
  {
    // Clear the entire frame buffer
    Ssd1306_ClearScreen();
    return;
  }

  // This is horribly inefficient, but at least easy to understand
  // In a production environment, this should be significantly optimised

  uint8_t y, x;
  for (y = 0; y < SSD1306_LCDHEIGHT; y++)
  {
    for (x = 0; x < SSD1306_LCDWIDTH; x++)
    {
      if ((SSD1306_LCDHEIGHT - 1) - y > height)
      {
        // Shift height from further ahead in the buffer
        Ssd1306_GetPixel(x, y + height) ? Ssd1306_DrawPixel(x, y) : Ssd1306_ClearPixel(x, y);
      }
      else
      {
        // Clear the entire line
        Ssd1306_ClearPixel(x, y);
      }
    }
  }
}
