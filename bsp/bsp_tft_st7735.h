#ifndef __BSP_TFT_ST7735_H_
#define __BSP_TFT_ST7735_H_

#include "main.h"

void bsp_LCD_Trans(uint8_t data);
void bsp_LCD_Quick_Trans(uint8_t data);


void WriteCmd(uint8_t dat);
void WriteDat(uint8_t dat);
void Write2Dat (uint16_t data);
void setPos(uint8_t sx, uint8_t ex, uint8_t sy, uint8_t ey);
void WriteAnColor(uint16_t color);
void LCD_Init(void);
void LCD_Flush_Color(uint16_t color);
void DMA_Color_Fill(uint8_t sx, uint8_t ex, uint8_t sy, uint8_t ey, uint16_t pColor);
void gram_set (uint16_t color, uint16_t buffersize);

#endif

