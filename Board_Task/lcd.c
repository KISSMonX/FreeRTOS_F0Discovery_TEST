#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "lcd.h"
#include "fonts.h"

/*-----------------------------------------------------------*/

/* LCD控制端口宏定义 */
#define LCD_GPIO_BL	GPIO_Pin_0//背光
#define LCD_GPIO_DC   	GPIO_Pin_1//DC：高电平为数据；低电平为命令
#define LCD_GPIO_CE   	GPIO_Pin_2//CE ：低电平使能LCD
#define LCD_GPIO_RST	GPIO_Pin_3//RST：低电平复位

#define LCD_RST_H	GPIO_SetBits(GPIOC, LCD_GPIO_RST)//RST置高，LCD正常工作
#define LCD_RST_L	GPIO_ResetBits(GPIOC, LCD_GPIO_RST)//RST置低，复位LCD

#define LCD_CE_H	GPIO_SetBits(GPIOC, LCD_GPIO_CE)//CE置高，禁用LCD
#define LCD_CE_L	GPIO_ResetBits(GPIOC, LCD_GPIO_CE)//CE置低，使能LCD
#define LCD_BL_H	GPIO_SetBits(GPIOC, LCD_GPIO_BL)//打开LCD背光
#define LCD_BL_L	GPIO_ResetBits(GPIOC, LCD_GPIO_BL)//关闭LCD背光
#define LCD_DC_DATA	GPIO_SetBits(GPIOC, LCD_GPIO_DC)//设置LCD为数据状态
#define LCD_DC_CMD	GPIO_ResetBits(GPIOC, LCD_GPIO_DC)//设置LCD为命令状态

/*-----------------------------------------------------------*/

extern uint8_t u8LCDFrameBuffer[LCD_X*LCD_Y/8];

/*-----------------------------------------------------------*/

/**
  * @brief  以SPI+DMA方式向LCD写入数据或者命令.
  * @param  ptr: RAM缓冲区指针
  * @param  len: 待写入的数据长度.
  * @retval 无
  */
void LCD_Send(uint32_t ptr, uint16_t len)
{
	if (DMA_GetCurrDataCounter(DMA1_Channel5) == 0)
	{		
		DMA_Cmd(DMA1_Channel5, DISABLE);//禁用DMA传输		
		DMA1_Channel5->CMAR = ptr;//设置RAM缓冲区指针		
		DMA1_Channel5->CNDTR = len;//设置数据缓冲区长度		
		DMA_Cmd(DMA1_Channel5, ENABLE );//使能DMA传输
	}	
	while(DMA_GetCurrDataCounter(DMA1_Channel5));//等待数据传输完成
}

/**
  * @brief  LCD清屏（显示缓冲区写入0）.
  * @param  无
  * @retval 无
  */
void LCD_Clear(void)
{
	uint16_t i;
	//显示缓冲区清零
	for(i = 0; i < LCD_X*LCD_Y/8; i++)
		u8LCDFrameBuffer[i] = 0;
}

/**
  * @brief  向LCD显示缓冲区中写入字符.
  * @param  ch: 待写入字符 
  * @param	X:	行 
  * @param	Y：列 
  * @retval 无
  */
void LCD_WriteChar(int8_t ch, uint8_t *X, uint8_t *Y)
{
	uint8_t n;
	//检查显示坐标X和Y
	LCD_XY_BoundsCheck(X, Y);
	//在显示字符前添加一个空格
	u8LCDFrameBuffer[((*X)++)+(*Y*LCD_X)] = 0x00;
	LCD_XY_BoundsCheck( X, Y );
	//将显示字符写入缓冲区
	for(n = 0; n < 5; n++)
	{
		u8LCDFrameBuffer[((*X)++)+(*Y*LCD_X)] = u8Font8x5[ch-0x20][n];
		LCD_XY_BoundsCheck(X, Y);
	}
	//在显示字符末尾添加一个空格
	u8LCDFrameBuffer[((*X)++)+(*Y*LCD_X)] = 0x00;
	LCD_XY_BoundsCheck(X, Y);
}

/**
  * @brief  检查XY坐标范围.
  * @param	*X:	行坐标地址
  * @param	*Y:	列坐标地址 
  * @retval 无
  */
void LCD_XY_BoundsCheck(uint8_t *X, uint8_t *Y)
{
	//如果到达最后一列，则从第一列开始显示
	if (*X > LCD_X)
	{
		*X = 0;
		*Y += 1;
	}
	//如果到达最后一行，则从第一行开始显示
	if (*Y >= LCD_Y/8)
	{
		*Y = 0;
	}
}

/**
  * @brief  向LCD显示缓冲区写入字符串.
  * @param  X: 行.
  * @param  Y: 列.  
  * @param  *s: 待写入字符串数组指针 
  * @retval 无
  */
void LCD_WriteString(uint8_t X, uint8_t Y, int8_t *s)
{
	//将待写入字符串写入显示缓冲区
	while (*s)
	{
		LCD_WriteChar(*s++, &X, &Y);
	}
}

/**
  * @brief  LCD5110初始化为SPI+DMA传输.
  * @param  无
  * @retval 无
  */
void LCD_Init(void)
{
	uint8_t LCDInitCmdBuffer[6] = {0x21, 0xD0, 0x04, 0x14, 0x20, 0x0C};
	
	/* LCD硬件复位 */
	LCD_DC_DATA;//切换到数据模式
	LCD_CE_H;//CE置高，禁用LCD
	
	LCD_RST_H;//RST置高
	Delay(1);//延时
	LCD_RST_L;//RST置低，复位LCD
	Delay(5);//延时20ms
	LCD_RST_H;//RST置高
	Delay(1);//延时
	
	LCD_CE_L;//CE置低，使能LCD
	LCD_DC_CMD;//切换到命令模式
	
	/* 
	* 初始化LCD 
	* 0x21：使用LCD扩展指令集
	* 0xD0：设置LCD对比度
	* 0x06：温度校正
	* 0x13：1:48
	* 0x20：使用基本指令集
	* 0x0C：普通模式
	*/
	LCD_Send((uint32_t)LCDInitCmdBuffer, 6);		
	Delay(10);//不加延时会导致液晶无法显示。(可能是液晶的响应时间与SPI写入速度不匹配)
	LCD_DC_DATA;//切换回数据模式
//	Delay(1);//延时10ms		
} 

void Delay(uint32_t t) 
{
	uint32_t j;
	for(; t>0; t--)
	{
		for(j=720000; j>0; j--);
	}
}
