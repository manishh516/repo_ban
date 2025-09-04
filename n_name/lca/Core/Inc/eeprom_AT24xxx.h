/**
  ******************************************************************************
  * @file           : eeprom_AT24xxx.h
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

#ifndef _EEPROM__AT24XXX_H
#define _EEPROM__AT24XXX_H


#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_i2c.h"

/***************************************************************************************************
                             Commonly used Eeprom macros/Constants
***************************************************************************************************/
#define AT24C01  128u														/* 1024  bits = 128    Bytes*/
#define AT24C02  256u														/* 2048  bits = 256    Bytes*/
#define AT24C04  512u														/* 4096  bits = 512    Bytes*/
#define AT24C08  1024u														/* 8192  bits = 1024  Bytes*/
#define AT24C16  2048u														/* 16384  bits = 2048  Bytes*/
#define AT24C32  4096u														/* 32768  bits = 4096  Bytes*/
#define AT24C64  8192u														/* 65536  bits = 8192  Bytes*/
#define AT24C128 16384u														/* 128K   bits = 16384 Bytes*/
#define AT24C256 32768u														/* 256K   bits = 32768 Bytes*/


#define C_EepromIdWriteMode_U8 0xA0u
#define C_EepromIdReadMode_U8  0xA1u 
/**************************************************************************************************/




/***************************************************************************************************
                 PreCompile configurations to enable/disable the functions
****************************************************************************************************
PreCompile configuration to enable or disable the API's.
 1.Required interfaces can be enabled/disabled by configuring its respective macros to 1/0.
 2. By default all the API's are disabled. 
***************************************************************************************************/
#define		ENABLE_EEPROM_WriteNBytes   1
#define		ENABLE_EEPROM_ReadNBytes    1
#define		ENABLE_EEPROM_WriteString   1
#define		ENABLE_EEPROM_ReadString    1
#define		ENABLE_EEPROM_Erase         1
#define		ENABLE_UART_CONFIG			0		/* Used For UART Config in RTU Slave Devices */
/**************************************************************************************************/


/***************************************************************************************************
                             Function Prototypes
****************************************************************************************************/
void EEPROM_Init				(uint16_t v_eepromType_u16);
void EEPROM_WriteByte		(uint16_t v_eepromAddress_u16, uint8_t v_eepromData_u8);
uint8_t EEPROM_ReadByte	(uint16_t v_eepromAddress_u16);
void EEPROM_WriteNBytes	(uint16_t v_eepromAddress_u16, uint8_t *ptr_ramAddress_u8, uint16_t v_numOfBytes_u16);
void EEPROM_ReadNBytes	(uint16_t v_eepromAddress_16, uint8_t *ptr_ramAddress_u8, uint16_t v_numOfBytes_u16);
void EEPROM_WriteString	(uint16_t v_eepromAddress_u16, uint8_t *ptr_string_u8);
void EEPROM_ReadString	(uint16_t v_eepromAddress_u16, uint8_t *ptr_destStringAddress_u8);
void EEPROM_Erase				(void);

/**
  * @brief  Read user setting from EEPROM
  * @name   readUserSetting
	* @param  uint8_t *rBuff
  * @retval None
  */
void readUserSetting(uint8_t *rBuff);

/**
  * @brief  Write default setting to EEPROM
  * @name   writeDefaultSetting
  * @param  None
  * @retval None
  */
void writeDefaultSetting(void);
	
/**
  * @brief  Write user setting to EEPROM
  * @name   writeUserSetting
  * @param  uint8_t *tBuff
  * @retval None
  */
void writeUserSetting(uint8_t *tBuff);

/**************************************************************************************************/

#endif

/***************************************************************************************************

    AT24xxx eeprom has two different memory addressing depending on the size of the.
    At24C01-At24C16 have single byte memory address where as AT24C32 to AT24C256 have 2-byte address.

    Further the AT24xxx series eeproms have the memory structure divided into pages.
    Size of the page depends on the IC series ex:At24C04 has 8-byte page where as AT24C04 has 16-bytes.

    -----------------------------------------------------------------------------------------
                        SingleByteAddress: AT24C01 - AT24C16
    ----------------------------------------------------------------------------------------*
    For single byte memory address, the actual address is obtained by the below formula.
    Actual page address = PageNumber * PageSize + Offset.
    ex: if address is 700(2*256 + 188) then the actual memory location is 188th byte of page 2(PageSize = 256)
    For a 16bit address d8-d10 represents the Page number and d0-d7 the byte number in that page.

    The page number has to be given as the part of eeprom Id(d1-d3)
    The Byte number within the page should be the address, which is sent after the device ID
    ------------------------------------------------------------------------------------------

    -----------------------------------------------------------------------------------------
                        DoubleByteAddress: AT24C32 - AT24C256
    -----------------------------------------------------------------------------------------
     For Double byte address, the 16bit address has to be sent as two bytes first MSB(D15 - D8)
     followed by LSB(D7 - D0)
    -----------------------------------------------------------------------------------------


    Note: User doesn't have to worry about the address decoding as the library takes care of it.
          It is just an info, so that proper IC needs to be selected from the eeprom.h file.        

***************************************************************************************************/
