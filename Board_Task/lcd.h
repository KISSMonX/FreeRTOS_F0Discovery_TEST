#ifndef __LCD_H
#define __LCD_H

#include "stm32f0xx.h"

#define LCD_X	84
#define LCD_Y	48

void LCD_Send(uint32_t ptr, uint16_t len);
void LCD_Init(void);
void LCD_Clear(void);
void LCD_WriteChar(int8_t ch, uint8_t *X, uint8_t *Y);
void LCD_XY_BoundsCheck(uint8_t *X, uint8_t *Y);
void LCD_WriteString(uint8_t X, uint8_t Y, int8_t *s);

void Delay(uint32_t t);
#endif /* __LCD_H */
