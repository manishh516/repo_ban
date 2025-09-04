#include "main.h"
#include "stm32f0xx_hal.h"
#include <stdio.h>
#include <string.h>

#define RS                  LCD_RS_Pin
#define RS_Port             LCD_RS_GPIO_Port

#define EN                  LCD_EN_Pin
#define EN_Port             LCD_EN_GPIO_Port

#define D4                  LCD_D4_Pin
#define D4_Port             LCD_D4_GPIO_Port

#define D5                  LCD_D5_Pin
#define D5_Port             LCD_D5_GPIO_Port

#define D6                  LCD_D6_Pin
#define D6_Port             LCD_D6_GPIO_Port

#define D7                  LCD_D7_Pin
#define D7_Port             LCD_D7_GPIO_Port


void LCD_Enable(void);

void LCD_Send4Bit(unsigned char Data);

void LCD_SendCommand(unsigned char command);

void LCD_Clear(void);

void LCD_Init(void);

void LCD_Gotoxy(unsigned char x, unsigned char y);

void LCD_PutChar(unsigned char Data);

void LCD_Puts(char *s);

void LCD_UPDATE(char *tBuff);
