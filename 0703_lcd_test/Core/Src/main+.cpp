/*
 * main+.cpp
 *
 *  Created on: Apr 11, 2024
 *      Author: JerryFu
 */

#include "main.h"

#include <stm32g4xx_ll_spi.h>
#include "tft18.h"
#include "Adafruit_ST7789.h"
//#include "Fonts/FreeSerif12pt7b.h"

using SpiCmd = Adafruit_SPITFT::SpiCmd;

void lcd_callback(SpiCmd cmd, uint8_t* pdata, size_t size)
{
  auto spi = hspi1.Instance;
  switch (cmd)
  {
  case SpiCmd::init:
    break;

  case SpiCmd::reset:
    break;

  case SpiCmd::cs_low:
    break;

  case SpiCmd::cs_high:
    break;

  case SpiCmd::dc_low:
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
    break;

  case SpiCmd::dc_high:
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
    break;

  case SpiCmd::transmit:
    HAL_SPI_Transmit(&hspi1, pdata, size, size);

//    LL_SPI_Enable(spi);
//    for (auto end = pdata + size; pdata != end; ++pdata)
//    {
//      LL_SPI_TransmitData8(spi, *pdata);
//      while (!LL_SPI_IsActiveFlag_TXE(spi))
//        ;
//    }
//    while (LL_SPI_IsActiveFlag_BSY(spi))
//      ;
//    LL_SPI_Disable(spi);

    break;

//  case SpiCmd::transmit_repeat:
//    LL_SPI_Enable(spi);
//    for (size_t i = 0; i != size; ++i)
//    {
//      LL_SPI_TransmitData16(spi, *(uint16_t*)pdata);
//      while (!LL_SPI_IsActiveFlag_TXE(spi))
//        ;
//    }
//    while (LL_SPI_IsActiveFlag_BSY(spi))
//      ;
//    LL_SPI_Disable(spi);
//    break;

//  case SpiCmd::transmit_byterev:
//    LL_SPI_Enable(spi);
//    for (auto end = pdata + size * 2; pdata != end; pdata += 2)
//    {
//      LL_SPI_TransmitData8(spi, *(pdata+1));
//      while (!LL_SPI_IsActiveFlag_TXE(spi))
//        ;
//      LL_SPI_TransmitData8(spi, *pdata);
//      while (!LL_SPI_IsActiveFlag_TXE(spi))
//        ;
//    }
//    while (LL_SPI_IsActiveFlag_BSY(spi))
//      ;
//    LL_SPI_Disable(spi);
//    break;

  case SpiCmd::delay:
    HAL_Delay(size);
    break;
  }
}

Adafruit_ST7789 lcd(240, 320, lcd_callback);

void setup()
{
  HAL_Delay(500);
  lcd.init(240, 320);
  lcd.setRotation(1);
  lcd.fillScreen(ST77XX_BLACK);
  lcd_show_picture(0, 0, 50, 75, gImage_elec);
  lcd.setCursor(115, 5);
  lcd.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  lcd.setTextSize(2);
  lcd.printf("1.AMPL\n");
  lcd_show_picture(101, 25, 12, 20, MenuCursor16x16);
  lcd.setCursor(115 , 25);
  lcd.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  lcd.printf("2.MODULATION\n");
  lcd.setCursor(115, 45);
  lcd.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  lcd.printf("3.DELAY\n");
  lcd.setCursor(115, 65);
  lcd.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  lcd.printf("4.ATTENUATION\n");
  lcd.setCursor(115, 85);
  lcd.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  lcd.printf("5.FREQUENCY\n");
  lcd.setCursor(115, 105);
  lcd.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  lcd.printf("6.INIT PHASE\n");
  lcd.setTextColor(ST77XX_RED, ST77XX_BLACK);

  //lcd.printf("Jerry's embedded\n");
//  for (int i = 0; i != 120; ++i)
//  {
//    lcd.drawFastHLine(i, 80+i, 240-2*i, ST77XX_RED);
//    lcd.drawFastHLine(i, 320-1-i, 240-2*i, ST77XX_YELLOW);
//    lcd.drawFastVLine(i, 80+i, 240-2*i, ST77XX_GREEN);
//    lcd.drawFastVLine(240-1-i, 80+i, 240-2*i, ST77XX_BLUE);
//    HAL_Delay(100);
//  }
}

void lcd_show_picture(uint16_t x, uint16_t y , uint16_t col , uint16_t row , const unsigned char * p)
{
	uint16_t * data = (uint16_t *) p;
	for (int i = y ; i < row + y ; ++i)
	{
		for ( int j = x ; j < col + x ; ++j)
		{
			lcd.drawPixel(j, i, *data++);
		}
	}
}
void lcd_show_num(int n)
{
	lcd.fillRect(150, 165, 230, 20, BLACK);
	lcd.setCursor(150, 165);
	lcd.printf("   ");
	lcd.setCursor(150, 165);
	lcd.print(n);
}
void lcd_show_str(int16_t x, int16_t y,const char * lmf)
{
	lcd.setCursor(x, y);
	lcd.fillRect(x, y, 230, 20, BLACK);
	lcd.printf(lmf);
}
void lcd_show_black(int16_t x, int16_t y , int16_t w, int16_t h)
{
	lcd.fillRect(x, y, w, h, BLACK);
}
void loop()
{
  ;
}

