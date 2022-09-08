#include "bsp_tft_st7735.h"
/* CubeMx Include */
#include "spi.h"
#include "lvgl.h"

/*LCD_BL背光*/
#define LCD_BL_ON()        HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_SET)
#define LCD_BL_OFF()       HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_RESET)
/*LCD_CS片选*/
#define LCD_CS_End()       HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET)
#define LCD_CS_Start()     HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET)
/*LCD_RESET*/
#define LCD_RES_SET()      HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_SET)
#define LCD_RES_RESET()    HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_RESET)
/*LCD_DC*/
#define LCD_WR_DAT()       HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET)
#define LCD_WR_CMD()       HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET)

#define __spi_speed        SPI_BAUDRATEPRESCALER_2  

//#define Gramsize 16384
//uint16_t Gram[Gramsize];

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_tx;

typedef struct __Anime
{
	bool flush_tg;
}Anime;

Anime St7735_Ani;




/*
*********************************************************************************************************
*    函 数 名: bsp_LCD_Quick_Trans
*    功能说明: 快速执行硬件spi 用于lcd初始化以及命令行配置
*    形    参: data
*    返 回 值: 无
*********************************************************************************************************
*/
void bsp_LCD_Quick_Trans(uint8_t data)
{
	SPI1->CFG1 = __spi_speed | SPI_DATASIZE_8BIT;
    SPI1->CR1 = SPI_CR1_SSI;
    SPI1->CR2 = 1;
    SPI1->CR1 = SPI_CR1_SPE | SPI_CR1_SSI;
    SPI1->CR1 = SPI_CR1_SPE | SPI_CR1_SSI | SPI_CR1_CSTART;
	
	/* Wait for TXE */
    while ((SPI1->SR & SPI_FLAG_TXE) == 0);

    *((__IO uint8_t *)&SPI1->TXDR) = data;
	
    while ((SPI1->SR & SPI_SR_TXC) == 0);
	
    SPI1->IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC;
	
	SPI1->CR1 &= ~(SPI_CR1_SPE);
}

/*
*********************************************************************************************************
*    函 数 名: WriteCmd & WriteDat & Write2Dat & setPos & WriteAnColor
*    功能说明: 主要作用与LCD配置与控制，利用bsp_LCD_Quick_Trans进行传输
*    形    参: ....
*    返 回 值: 无
*********************************************************************************************************
*/
void WriteCmd(uint8_t dat)
{
	LCD_CS_Start();
	LCD_WR_CMD();
	bsp_LCD_Quick_Trans(dat);
	LCD_CS_End();
}

void WriteDat(uint8_t dat)
{
	LCD_CS_Start();
	LCD_WR_DAT();
	bsp_LCD_Quick_Trans(dat);
	LCD_CS_End();
}

void Write2Dat (uint16_t data)
{
	WriteDat(data>>8);
	WriteDat(data);
}

//定位
void setPos( uint8_t sx, uint8_t ex, uint8_t sy, uint8_t ey ) 
{
	// X轴
	// 注意这里的“sx+2”，就是之前所说的偏移问题的解决方法
	WriteCmd(0x2a);
	WriteDat(0x00);
	WriteDat(sx+2);
	WriteDat(0x00);
	WriteDat(ex+2);
	// Y轴
	WriteCmd(0x2b);
	WriteDat(0x00);
	WriteDat(sy+3);
	WriteDat(0x00);
	WriteDat(ey+3);
	// 很重要！最后的写入命令
	WriteCmd(0x2c);
}

void WriteAnColor(uint16_t color)
{
	uint8_t x=0;
	uint8_t y=0;
	setPos( 0, 127, 0, 127 );
	for ( y = 0; y < 128; y++ ) 
	{
		for ( x = 0; x < 128; x++ ) 
		{
			Write2Dat( color );
		}
	}
}


/*
*********************************************************************************************************
*    函 数 名: LCD_Init
*    功能说明: LCD初始化
*    形    参: None
*    返 回 值: None
*********************************************************************************************************
*/

void LCD_Init(void)
{
	LCD_BL_OFF();
	LCD_RES_RESET();
	HAL_Delay(500);
	LCD_RES_SET();
	HAL_Delay(500);
	
//	WriteCmd(0x2C);
	WriteCmd(0x28);
	WriteCmd(0x11);//Sleep exit 
	
	HAL_Delay(500);
	
	//ST7735R Frame Rate
//	WriteCmd(0xB1); 
//	WriteDat(0x01); 
//	WriteDat(0x2C); 
//	WriteDat(0x2D); 

	WriteCmd(0xB1); 
    WriteDat(0x01); 
	WriteDat(0x2C); 
	WriteDat(0x2C); 


	WriteCmd(0xB2); 
	WriteDat(0x01); 
	WriteDat(0x2C); 
	WriteDat(0x2C);

	WriteCmd(0xB3); 
	WriteDat(0x01); 
	WriteDat(0x2C); 
	WriteDat(0x2C);
	WriteDat(0x01); 
	WriteDat(0x2C); 
	WriteDat(0x2C); 
	
	WriteCmd(0xB4); //Column inversion 
	WriteDat(0x07); 
	
	//ST7735R Power Sequence
	WriteCmd(0xC0); 
	WriteDat(0xA2); 
	WriteDat(0x02); 
	WriteDat(0x84); 
	WriteCmd(0xC1); 
	WriteDat(0xC5); 

	WriteCmd(0xC2); 
	WriteDat(0x0A); 
	WriteDat(0x00); 

	WriteCmd(0xC3); 
	WriteDat(0x8A); 
	WriteDat(0x2A); 
	WriteCmd(0xC4); 
	WriteDat(0x8A); 
	WriteDat(0xEE); 
	
	WriteCmd(0xC5); //VCOM 
	WriteDat(0x0E); 
	
	WriteCmd(0x36); //MX, MY, RGB mode 
	WriteDat(0xC8); //竖屏C8 横屏08 A8	
	
	//ST7735R Gamma Sequence
	WriteCmd(0xe0); 
	WriteDat(0x0f); 
	WriteDat(0x1a); 
	WriteDat(0x0f); 
	WriteDat(0x18); 
	WriteDat(0x2f); 
	WriteDat(0x28); 
	WriteDat(0x20); 
	WriteDat(0x22); 
	WriteDat(0x1f); 
	WriteDat(0x1b); 
	WriteDat(0x23); 
	WriteDat(0x37); 
	WriteDat(0x00); 	
	WriteDat(0x07); 
	WriteDat(0x02); 
	WriteDat(0x10); 

	WriteCmd(0xe1); 
	WriteDat(0x0f); 
	WriteDat(0x1b); 
	WriteDat(0x0f); 
	WriteDat(0x17); 
	WriteDat(0x33); 
	WriteDat(0x2c); 
	WriteDat(0x29); 
	WriteDat(0x2e); 
	WriteDat(0x30); 
	WriteDat(0x30); 
	WriteDat(0x39); 
	WriteDat(0x3f); 
	WriteDat(0x00); 
	WriteDat(0x07); 
	WriteDat(0x03); 
	WriteDat(0x10);  
	
	WriteCmd(0xF0); //Enable test command  
	WriteDat(0x01); 
	WriteCmd(0xF6); //Disable ram power save mode 
	WriteDat(0x00); 
	
	WriteCmd(0x3A); //65k mode 
	WriteDat(0x05); 
	
	WriteCmd(0x29);//Display on
	
	LCD_BL_ON();
	WriteAnColor(0x0000);
	
}

/*
*********************************************************************************************************
*    函 数 名: LCD_Flush_Color
*    功能说明: 刷屏函数 spi+dma
*    形    参: color(rgb565 16bits)
*    返 回 值: None
*********************************************************************************************************
*/
//void LCD_Flush_Color(uint16_t color)
//{
//    int i=0;
//	for(i=0;i<Gramsize;i++) Gram[i]=color;
//	setPos( 0, 127, 0, 127 );
//	
//	SPI1->CFG1 = __spi_speed | SPI_DATASIZE_16BIT;
//	LCD_CS_Start();
//	LCD_WR_DAT();
//	HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)Gram, Gramsize);
//}

//void gram_set (uint16_t color, uint16_t buffersize)
//{
//	int i=0;
//	for(i=0;i<buffersize;i++) Gram[i]=color;
//}

void DMA_Color_Fill(uint8_t sx, uint8_t ex, uint8_t sy, uint8_t ey, uint16_t* pColor)
{
	/* Count buffersize */
	uint16_t width,height;
	uint16_t buffersize;
	width=ex-sx+1; 			//宽度
	height=ey-sy+1;			//高度
	buffersize=width*height;
	
	/* Set Position of flush area */
	setPos(sx, ex, sy, ey);
	
	/* Set CFG1 & Callback */
	SPI1->CFG1 = __spi_speed | SPI_DATASIZE_16BIT;
	
	/* Start Transmit */
	LCD_CS_Start();
	LCD_WR_DAT();
	HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)pColor, buffersize);
}




/*
*********************************************************************************************************
*    函 数 名: Callback_Set & SPI_TxCpltCallback
*    功能说明: 回调函数定向 & SPI传输完成回调函数，在传输完成中断中关闭片选
*    形    参: None
*    返 回 值: None
*********************************************************************************************************
*/


void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	UNUSED(hspi);
	LCD_CS_End();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	UNUSED(htim);
	lv_tick_inc(1);
}
