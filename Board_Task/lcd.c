#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "lcd.h"
#include "fonts.h"

/*-----------------------------------------------------------*/

/* LCD���ƶ˿ں궨�� */
#define LCD_GPIO_BL	GPIO_Pin_0//����
#define LCD_GPIO_DC   	GPIO_Pin_1//DC���ߵ�ƽΪ���ݣ��͵�ƽΪ����
#define LCD_GPIO_CE   	GPIO_Pin_2//CE ���͵�ƽʹ��LCD
#define LCD_GPIO_RST	GPIO_Pin_3//RST���͵�ƽ��λ

#define LCD_RST_H	GPIO_SetBits(GPIOC, LCD_GPIO_RST)//RST�øߣ�LCD��������
#define LCD_RST_L	GPIO_ResetBits(GPIOC, LCD_GPIO_RST)//RST�õͣ���λLCD

#define LCD_CE_H	GPIO_SetBits(GPIOC, LCD_GPIO_CE)//CE�øߣ�����LCD
#define LCD_CE_L	GPIO_ResetBits(GPIOC, LCD_GPIO_CE)//CE�õͣ�ʹ��LCD
#define LCD_BL_H	GPIO_SetBits(GPIOC, LCD_GPIO_BL)//��LCD����
#define LCD_BL_L	GPIO_ResetBits(GPIOC, LCD_GPIO_BL)//�ر�LCD����
#define LCD_DC_DATA	GPIO_SetBits(GPIOC, LCD_GPIO_DC)//����LCDΪ����״̬
#define LCD_DC_CMD	GPIO_ResetBits(GPIOC, LCD_GPIO_DC)//����LCDΪ����״̬

/*-----------------------------------------------------------*/

extern uint8_t u8LCDFrameBuffer[LCD_X*LCD_Y/8];

/*-----------------------------------------------------------*/

/**
  * @brief  ��SPI+DMA��ʽ��LCDд�����ݻ�������.
  * @param  ptr: RAM������ָ��
  * @param  len: ��д������ݳ���.
  * @retval ��
  */
void LCD_Send(uint32_t ptr, uint16_t len)
{
	if (DMA_GetCurrDataCounter(DMA1_Channel5) == 0)
	{		
		DMA_Cmd(DMA1_Channel5, DISABLE);//����DMA����		
		DMA1_Channel5->CMAR = ptr;//����RAM������ָ��		
		DMA1_Channel5->CNDTR = len;//�������ݻ���������		
		DMA_Cmd(DMA1_Channel5, ENABLE );//ʹ��DMA����
	}	
	while(DMA_GetCurrDataCounter(DMA1_Channel5));//�ȴ����ݴ������
}

/**
  * @brief  LCD��������ʾ������д��0��.
  * @param  ��
  * @retval ��
  */
void LCD_Clear(void)
{
	uint16_t i;
	//��ʾ����������
	for(i = 0; i < LCD_X*LCD_Y/8; i++)
		u8LCDFrameBuffer[i] = 0;
}

/**
  * @brief  ��LCD��ʾ��������д���ַ�.
  * @param  ch: ��д���ַ� 
  * @param	X:	�� 
  * @param	Y���� 
  * @retval ��
  */
void LCD_WriteChar(int8_t ch, uint8_t *X, uint8_t *Y)
{
	uint8_t n;
	//�����ʾ����X��Y
	LCD_XY_BoundsCheck(X, Y);
	//����ʾ�ַ�ǰ���һ���ո�
	u8LCDFrameBuffer[((*X)++)+(*Y*LCD_X)] = 0x00;
	LCD_XY_BoundsCheck( X, Y );
	//����ʾ�ַ�д�뻺����
	for(n = 0; n < 5; n++)
	{
		u8LCDFrameBuffer[((*X)++)+(*Y*LCD_X)] = u8Font8x5[ch-0x20][n];
		LCD_XY_BoundsCheck(X, Y);
	}
	//����ʾ�ַ�ĩβ���һ���ո�
	u8LCDFrameBuffer[((*X)++)+(*Y*LCD_X)] = 0x00;
	LCD_XY_BoundsCheck(X, Y);
}

/**
  * @brief  ���XY���귶Χ.
  * @param	*X:	�������ַ
  * @param	*Y:	�������ַ 
  * @retval ��
  */
void LCD_XY_BoundsCheck(uint8_t *X, uint8_t *Y)
{
	//����������һ�У���ӵ�һ�п�ʼ��ʾ
	if (*X > LCD_X)
	{
		*X = 0;
		*Y += 1;
	}
	//����������һ�У���ӵ�һ�п�ʼ��ʾ
	if (*Y >= LCD_Y/8)
	{
		*Y = 0;
	}
}

/**
  * @brief  ��LCD��ʾ������д���ַ���.
  * @param  X: ��.
  * @param  Y: ��.  
  * @param  *s: ��д���ַ�������ָ�� 
  * @retval ��
  */
void LCD_WriteString(uint8_t X, uint8_t Y, int8_t *s)
{
	//����д���ַ���д����ʾ������
	while (*s)
	{
		LCD_WriteChar(*s++, &X, &Y);
	}
}

/**
  * @brief  LCD5110��ʼ��ΪSPI+DMA����.
  * @param  ��
  * @retval ��
  */
void LCD_Init(void)
{
	uint8_t LCDInitCmdBuffer[6] = {0x21, 0xD0, 0x04, 0x14, 0x20, 0x0C};
	
	/* LCDӲ����λ */
	LCD_DC_DATA;//�л�������ģʽ
	LCD_CE_H;//CE�øߣ�����LCD
	
	LCD_RST_H;//RST�ø�
	Delay(1);//��ʱ
	LCD_RST_L;//RST�õͣ���λLCD
	Delay(5);//��ʱ20ms
	LCD_RST_H;//RST�ø�
	Delay(1);//��ʱ
	
	LCD_CE_L;//CE�õͣ�ʹ��LCD
	LCD_DC_CMD;//�л�������ģʽ
	
	/* 
	* ��ʼ��LCD 
	* 0x21��ʹ��LCD��չָ�
	* 0xD0������LCD�Աȶ�
	* 0x06���¶�У��
	* 0x13��1:48
	* 0x20��ʹ�û���ָ�
	* 0x0C����ͨģʽ
	*/
	LCD_Send((uint32_t)LCDInitCmdBuffer, 6);		
	Delay(10);//������ʱ�ᵼ��Һ���޷���ʾ��(������Һ������Ӧʱ����SPIд���ٶȲ�ƥ��)
	LCD_DC_DATA;//�л�������ģʽ
//	Delay(1);//��ʱ10ms		
} 

void Delay(uint32_t t) 
{
	uint32_t j;
	for(; t>0; t--)
	{
		for(j=720000; j>0; j--);
	}
}
