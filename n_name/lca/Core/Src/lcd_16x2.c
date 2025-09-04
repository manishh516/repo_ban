#include "main.h"
#include "stm32f0xx_hal.h"
#include "lcd_16x2.h"

extern int lcd_on_flag;

void LCD_Enable(void)
{
HAL_GPIO_WritePin(EN_Port,EN,GPIO_PIN_SET);
HAL_Delay(1);
HAL_GPIO_WritePin(EN_Port,EN,GPIO_PIN_RESET);	
HAL_Delay(1);	
}

void LCD_Send4Bit(unsigned char Data)
{
	if(Data&0x01){HAL_GPIO_WritePin(D4_Port,D4,GPIO_PIN_SET);}
	else         {HAL_GPIO_WritePin(D4_Port,D4,GPIO_PIN_RESET);}
	
	if((Data>>1)&0x01){HAL_GPIO_WritePin(D5_Port,D5,GPIO_PIN_SET);}
	else         {HAL_GPIO_WritePin(D5_Port,D5,GPIO_PIN_RESET);}
	
	if((Data>>2)&0x01){HAL_GPIO_WritePin(D6_Port,D6,GPIO_PIN_SET);}
	else         {HAL_GPIO_WritePin(D6_Port,D6,GPIO_PIN_RESET);}
	
	if((Data>>3)&0x01){HAL_GPIO_WritePin(D7_Port,D7,GPIO_PIN_SET);}
	else         {HAL_GPIO_WritePin(D7_Port,D7,GPIO_PIN_RESET);}

}

void LCD_SendCommand(unsigned char command)
{
	LCD_Send4Bit(command >>4);
	LCD_Enable();
	LCD_Send4Bit(command);
	LCD_Enable();
}

void LCD_Clear(void)
{
	if(lcd_on_flag==1)
		{
			LCD_SendCommand(0x01);  
	}
 // HAL_Delay(1);	
}

void LCD_Init(void)
{
	LCD_Send4Bit(0x00);
  HAL_GPIO_WritePin(RS_Port,RS,GPIO_PIN_RESET);
	LCD_Send4Bit(0x03);
	LCD_Enable();
	LCD_Enable();
	LCD_Enable();
	LCD_Send4Bit(0x02);
	LCD_Enable();
	LCD_SendCommand(0x28);
	LCD_SendCommand(0x0C);
	LCD_SendCommand(0x06);
	LCD_SendCommand(0x01);
}

void LCD_Gotoxy(unsigned char x, unsigned char y)
{
	if(lcd_on_flag==1)   //updated by rishabh
	{
		unsigned char address;
		if(y==0) {
			address=0x80;
		}
		else if(y==1){
			address=0xc0;
		} 
		else if(y==2){
			address=0x94;
		}
		else {
			address=0xd4;
		}
		
		address+=x;
		LCD_SendCommand(address);
	}
}

void LCD_PutChar(unsigned char Data)
{
  HAL_GPIO_WritePin(RS_Port,RS,GPIO_PIN_SET);
 	LCD_SendCommand(Data);
  HAL_GPIO_WritePin(RS_Port,RS,GPIO_PIN_RESET);
}

void LCD_Puts(char *s)
{
	
	if(lcd_on_flag==1)   //updated by rishabh
	{
		while (*s){
      	LCD_PutChar(*s);
     	s++;
   	}
	}
   	
}

void LCD_UPDATE(char *tBuff){
	int l = strlen(tBuff);
	for(int i =l; i<32; i++){
		tBuff[i] = ' ';
	}
	char tBuff_1[16];
	char tBuff_2[16];
	strncpy(tBuff_1,tBuff,16);
	strncpy(tBuff_2,tBuff+16,16);
	LCD_Gotoxy(0,0);
	LCD_Puts(tBuff_1);
	LCD_Gotoxy(0,1);
	LCD_Puts(tBuff_2);
	
	memset(tBuff, 0x00,32);
}
