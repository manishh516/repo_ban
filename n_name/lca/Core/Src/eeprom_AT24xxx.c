/**
  ******************************************************************************
  * @file           : eeprom_AT24xxx.c
  * @brief          : AT24xxx EEPROM Library for STM32
  * @Date           : 01 DEC 2019
  ******************************************************************************
  * Copyright (c) 2019 Suman Kumar Das <sumancvb@gmail.com>
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions
  * are met:
  * 1. Redistributions of source code must retain the above copyright
  *    notice, this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright
  *    notice, this list of conditions and the following disclaimer in the
  *    documentation and/or other materials provided with the distribution.
  * 3. The name of the author may not be used to endorse or promote products
  *    derived from this software without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
  * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
  * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#include "eeprom_AT24xxx.h"
#include "main.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_i2c.h"
#include <string.h>
#include <stdio.h>


extern I2C_HandleTypeDef hi2c1;
uint16_t V_EepromType_U16;


/**
  * @brief  Init EEPROM
  * @param  v_eepromType_u16: EEPROM Type
  * @retval None
  */
void EEPROM_Init(uint16_t v_eepromType_u16)
{
    V_EepromType_U16 = v_eepromType_u16;
}

/**
  * @brief  This function is used to write the data at specified EEPROM_address..
  * @param  v_eepromAddress_u16: EEPROM Address
  * @param  v_eepromData_u8: Data
  * @retval None
  */
void EEPROM_WriteByte(uint16_t v_eepromAddress_u16, uint8_t v_eepromData_u8)
{    

    uint8_t v_eepromLowerAddress_u8;
    uint8_t v_eepromPageNumber_u8;
    uint8_t v_eepromId_u8 = C_EepromIdWriteMode_U8;
    uint8_t v_eepromHigherAddress_u8;

    if(v_eepromAddress_u16 < V_EepromType_U16)  // Is Address within the Eeprom Limit   
    {
        v_eepromLowerAddress_u8 = v_eepromAddress_u16&0xFF;

       if (V_EepromType_U16 >  AT24C16)  // If Ic is greater tha 24C16 then double byte address needs to be supported.
         {
           v_eepromHigherAddress_u8 = v_eepromAddress_u16>>8;
         } 
       else
         {
		   /* Single byte address */
           v_eepromPageNumber_u8 = v_eepromAddress_u16>>8;
           v_eepromId_u8 = v_eepromId_u8 | (v_eepromPageNumber_u8 << 1);
         }           
       
       while(HAL_I2C_Mem_Write(&hi2c1,(uint16_t)v_eepromId_u8,(uint16_t)v_eepromLowerAddress_u8,I2C_MEMADD_SIZE_8BIT,&v_eepromData_u8,1,1000)!= HAL_OK);
    }    
}

/**
  * @brief  This function is used to read specified EEPROM_address.
  * @param  v_eepromAddress_u16: EEPROM Address
  * @retval Data
  */     
uint8_t EEPROM_ReadByte(uint16_t v_eepromAddress_u16)
{
    uint8_t v_eepromLowerAddress_u8;
    uint8_t v_eepromPageNumber_u8;
    uint8_t v_eepromData_u8 = 0x00;
    uint8_t v_eepromId_u8 = C_EepromIdReadMode_U8;
    uint8_t v_eepromHigherAddress_u8;

    if(v_eepromAddress_u16 < V_EepromType_U16) /* Is address within Eeprom Limit */   
    {
        v_eepromLowerAddress_u8 = v_eepromAddress_u16 & 0xFF;

       if(V_EepromType_U16 >  AT24C16)
       {
           v_eepromHigherAddress_u8 =  v_eepromAddress_u16 >> 8;
       } 
      else
       {
          v_eepromPageNumber_u8 =  v_eepromAddress_u16 >> 8;
          v_eepromId_u8 = v_eepromId_u8 | (v_eepromPageNumber_u8 << 1); 
       }
			while(HAL_I2C_Mem_Read(&hi2c1,(uint16_t)v_eepromId_u8,(uint16_t)v_eepromLowerAddress_u8,I2C_MEMADD_SIZE_8BIT,&v_eepromData_u8,(uint16_t)1,1000)!= HAL_OK );
    }
    return v_eepromData_u8;
}

/**
  * @brief  This function is used to write N-bytes of data at specified EEPROM_address.
  * @param  v_eepromAddress_u16: eeprom_address from where the N-bytes are to be written.
  * @param  *ptr_ramAddress_u8: Buffer containing the N-Bytes of data to be written in Eeprom.
  * @param  v_numOfBytes_u16: Number of bytes to be written
  * @retval None
  */
#if ( ENABLE_EEPROM_WriteNBytes == 1)
void EEPROM_WriteNBytes(uint16_t v_eepromAddress_u16, uint8_t *ptr_ramAddress_u8, uint16_t v_numOfBytes_u16)
{
    while(v_numOfBytes_u16 !=  0)
    {
        EEPROM_WriteByte(v_eepromAddress_u16, *ptr_ramAddress_u8); //Write a byte from RAM to EEPROM

			  if ('\0' == *(ptr_ramAddress_u8))
				{
					return;
				}
				v_eepromAddress_u16++;                    //Increment the Eeprom Address
        ptr_ramAddress_u8++;                      //Increment the RAM Address
        v_numOfBytes_u16--;                       //Decrement NoOfBytes after writing each Byte
    }
}
#endif

/**
  * @brief  This function is used to Read N-bytes of data from specified EEPROM_address.
  * @param  v_eepromAddress_16: eeprom_address from where the N-bytes is to be read.
  * @param  *ptr_ramAddress_u8: Pointer to copy the N-bytes read from Eeprom.
  * @param  v_numOfBytes_u16: Number of bytes to be Read
  * @retval None
  */
#if ( ENABLE_EEPROM_ReadNBytes == 1)
void EEPROM_ReadNBytes(uint16_t v_eepromAddress_16, uint8_t *ptr_ramAddress_u8, uint16_t v_numOfBytes_u16)
{
    while(v_numOfBytes_u16 !=  0)
    {
        *ptr_ramAddress_u8 = EEPROM_ReadByte(v_eepromAddress_16);//Read a byte from EEPROM to RAM
			  if ('\0' == (*ptr_ramAddress_u8))
				{
				  return;
				}
        v_eepromAddress_16++;                        //Increment the EEPROM Address
        ptr_ramAddress_u8++;                            //Increment the RAM Address
        v_numOfBytes_u16--;                        //Decrement NoOfBytes after Reading each Byte
    }
}
#endif

/**
  * @brief  This function is used to Write a String at specified EEPROM_address.
  * @param  *ptr_string_u8: Pointer to String which has to be written.
  * @retval None
  * NOTE: Null char is also written into the eeprom.
  */
#if ( ENABLE_EEPROM_WriteString == 1)
void EEPROM_WriteString(uint16_t v_eepromAddress_u16, uint8_t *ptr_string_u8)
{
    do
    {
        EEPROM_WriteByte(v_eepromAddress_u16,*ptr_string_u8); //Write a byte from RAM to EEPROM
        ptr_string_u8++;                                //Increment the RAM Address
        v_eepromAddress_u16++;                                //Increment the Eeprom Address
    }while(*(ptr_string_u8-1) != '\0');
}
#endif

/**
  * @brief  This function is used to Read a String from specified EEPROM_address.
  * @param  v_eepromAddress_u16: eeprom_address from where the String is to be read.
  * @param  *ptr_destStringAddress_u8: Pointer into which the String is to be read.
  * @retval None
  * NOTE: The string read from eeprom will be copied to specified buffer along with NULL character
  */
#if ( ENABLE_EEPROM_ReadString == 1)
void EEPROM_ReadString(uint16_t v_eepromAddress_u16, uint8_t *ptr_destStringAddress_u8)
{
    char eeprom_data;

    do
    {
        eeprom_data = EEPROM_ReadByte(v_eepromAddress_u16); //Read a byte from EEPROM to RAM
        *ptr_destStringAddress_u8 = eeprom_data;             //Copy the data into String Buffer
        ptr_destStringAddress_u8++;                         //Increment the RAM Address
        v_eepromAddress_u16++;                             //Increment the Eeprom Address
    }while(eeprom_data!= '\0');
}
#endif

/**
  * @brief  This function is used to erase the entire EEprom memory.
  * @param  None
  * @retval None
  * NOTE: Complete Eeprom(V_EepromType_U16) is filled with 0xFF to accomplish the Eeprom Erase.
  */
#if ( ENABLE_EEPROM_Erase == 1)
void EEPROM_Erase(void)
{
    uint16_t v_eepromAddress_u16;

    for(v_eepromAddress_u16=0; v_eepromAddress_u16 < V_EepromType_U16; v_eepromAddress_u16++)
    {
        EEPROM_WriteByte(v_eepromAddress_u16,0xFFu); // Write Each memory location with OxFF
    }
}
#endif

#if ( ENABLE_UART_CONFIG == 1)
/**
  * @brief  Read user setting from EEPROM
  * @name   readUserSetting
	* @param  uint8_t *rBuff
  * @retval None
  */
void readUserSetting(uint8_t *rBuff){
	rBuff [0] = EEPROM_ReadByte(0x0001);			//slaveID
	rBuff [1] = EEPROM_ReadByte(0x0002);			//BaudRate
	rBuff [2] = EEPROM_ReadByte(0x0003);			//Parity
	rBuff [3] = EEPROM_ReadByte(0x0004);			//Wordlength
	rBuff [4] = EEPROM_ReadByte(0x0005);			//StopBit
}

/**
  * @brief  Write default setting to EEPROM
  * @name   writeDefaultSetting
  * @param  None
  * @retval None
  */
void writeDefaultSetting(void){
	EEPROM_WriteByte(0x0001, 0x01U);			//slaveID
	EEPROM_WriteByte(0x0002, 0x03U);			//BaudRate
	EEPROM_WriteByte(0x0003, 0x01U);			//Parity
	EEPROM_WriteByte(0x0004, 0x01U);			//Wordlength
	EEPROM_WriteByte(0x0005, 0x01U);			//StopBit
}
	
/**
  * @brief  Write user setting to EEPROM
  * @name   writeUserSetting
  * @param  uint8_t *tBuff
  * @retval None
  */
void writeUserSetting(uint8_t *tBuff){						
	EEPROM_WriteByte(0x0001, tBuff[0]);			//slaveID
	EEPROM_WriteByte(0x0002, tBuff[1]);			//BaudRate
	EEPROM_WriteByte(0x0003, tBuff[2]);			//Parity
	EEPROM_WriteByte(0x0004, tBuff[3]);			//Wordlength
	EEPROM_WriteByte(0x0005, tBuff[4]);			//StopBit
}
#endif
