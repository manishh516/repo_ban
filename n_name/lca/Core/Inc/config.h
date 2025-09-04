/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : config.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONFIG_H
#define __CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */




//EEPROM Address
#define EP_ERRORFLAG                 0   // 1   //page 1
#define EP_PUMPSTATUS                1   // 1   //page 1
#define EP_PUMPINITSECS              2   // 1   //page 1
#define EP_RECHARGETIMEMINS          3   // 2   //page 1
#define EP_WEEKDAYS_PGMA             5   // 1   //page 1
#define EP_IRRG_MODE                 6   // 2   //page 1
#define EP_CURRSCHIDX                8   // 1   //page 1
#define EP_LASTINSTMINS              9   // 2   //page 1
#define EP_VALVECNT                 11   // 1
#define EP_EFFECTIVETODAY           12   // 1
#define EP_CONFIGURED               13   // 2
#define EP_WEEKDAYS_PGMB            15   // 1

#define EP_MASTER_PUMP_SCHD_STTIME              16  //8    //page 2
//#define EP_2ND_RT_DAY                           24  //1     //page 2
#define EP_FLOW_PULSE                           24  //1     //page 2
//#define EP_2ND_RT_INSTMINS                      25  //1     //page 2 // ori
#define EP_SCHEDULE_COMPLETE                    26  //6     //page 2

#define EP_PENDIRRG                             27  //1
#define EP_UNUSED                               28  //4     //page 2
//#define EP_TODAY_PUMP_SCHD_STTIME               24  //8     //page 2 -- in ori

#define EP_MASTER_VALVE_SCHD_STTIME             32  //96    //page 3, 4
#define EP_TODAY_VALVE_SCHD_STTIME              128  //96    //page 5, 6
#define EP_MASTER_VALVE_SCHD_DUR                224  //96    //page 7, 8

#define EP_MASTER_VALVE_PREWET_DUR              320  //96    //page 9, 10
#define EP_MASTER_VALVE_FERTG_DUR               416  //96    //page 11,12
#define EP_MASTER_VALVE_FLUSH_DUR               512  //96    //page 13,14

#define EP_TODAY_VALVE_SCHD_DUR                 608  //96    //page 15, 16

#define EP_TODAY_VALVE_PREWET_DUR              704  //96    //page 17, 18
#define EP_TODAY_VALVE_FERTG_DUR               800  //96    //page 19,20
#define EP_TODAY_VALVE_FLUSH_DUR               896  //96 //992    //page 21,22

#define EP_get_total_v													993   // @@ ##
//#define EP_get_exp    													994   //1  @@ ##
#define EP_pkt_t_STORE_BIT   										994   //1  @@ ## // pkt_t_store_bit
//#define EP_2ND_RT_INSTMINS                      995  //2  @@## 
#define EP_CUSTOM_PKT_TIME                      995  //2  @@##
#define EP_SIM_DISABLE_BIT                      999  //1  @@## sim_new
#define EP_server_IP        										1001   //16 @@ ##
#define EP_apn               										1017   //65 @@ ##
#define EP_IP_STORE        											1082   //1  //ip_bit_store
#define EP_APN_STORE       											1083   //1  //apn_bit_store 
#define EP_PASS_STORE      											1084   //1  //pass_bit_store
#define EP_PORT_STORE      											1085   //1  //port_bit_store
#define EP_PASS_CODE       											1086   //4
#define EP_PORT_NUM        											1091   //6  //port_bit_store
#define EP_RAIN_DISABLE_BIT        							1097   //1  //port_bit_store
#define EP_LF_DISABLE_BIT        								1098   //1  //port_bit_store
#define EP_WATER_M          										1100   //??


#define MAX_ERROR_CNT            8
#define OFF                      0
#define ON                       1
#define PGM_A                    1
#define PGM_B                    2
#define IRRIGATION_ONLY_MODE     1 // Irrigation Mode 
#define FERTIGATION_MODE         2// Irrigation cum Fertigation Mode

#define TOTAL_VALVES 12

#define STOP                    0
#define RUNNING                 1

#define PREWET       1
#define FERTIGATION  2
#define FLUSHING     3

#if 0
#define SW_START_SZ   4
#define SW_DAYS_SZ    8
#define SW_PGM_SEL_SZ 2
#define SW_PGM_SZ    20


#define SW_START        1
#define SW_DAYS_ST      SW_START+SW_START_SZ+1
#define SW_PGM_SEL_ST   SW_DAYS_ST+SW_DAYS_SZ
#define SW_PGM_A_ST     SW_PGM_SEL_ST+SW_PGM_SEL_SZ
#define SW_PGM_B_ST     SW_PGM_A_ST+SW_PGM_SZ
#else
#define SW_START_SZ   1//3
#define SW_DAYS_SZ    9
//#define SW_PGM_SEL_SZ 2 // @@
#define SW_PGM_SEL_SZ 4
#define SW_PGM_SZ    20


#define SW_START        1
#define SW_PGM_SEL_ST   SW_START+SW_START_SZ  //SW_DAYS_ST+SW_DAYS_SZ
#define SW_PGM_A_ST     SW_PGM_SEL_ST+SW_PGM_SEL_SZ  // 2+4
#define SW_PGM_B_ST     SW_PGM_A_ST+SW_PGM_SZ // 6+20
#endif
#define SWCNT_IRRG      24
#define SWCNT_FERT      36
#define SW_DELAY    1000


#define IRRIGATION_DATA 0x01
#define PUMP_DATA 0x02
#define CLOUD_DATA 0x03
// Configuration setting

#define MAX_VALVES 12 // Maximum number of valves // @@


// TimeOut if no activity for few secs (60 sec), use last saved config and go to execution mode
//	Stop all output signals (what about the running duration?)
//	Set/update Date and time 
      // View Date and Time

//	Check for Pump ON/OFF
      //	   If pump OFF, Recharging and pump initiation time will not be configured.


//	Select Program (A/B)  [Need to comeback to program B config, if Program A selected ]
      // Program A
         
				//	Select Days for selected program (it would show All days, selected one should have ENTER key pressed, for non-selected one �NEXT� key should be pressed)
				//	Select Start Time1
        //  Select Start Time2
				//  Select Mode (Irrigation/ Fertigation)
				//	Select config for all valves if Irrigation mode selected
				//    	Check configured parameters  doesn�t have any overlapping
				//  Select Pre-wet time, config for all valves if Fertigation mode selected
				//    	Check configured parameters doesn�t have any overlapping

      // Program B
				//	Select Days for selected program (it would show All days, selected one should have ENTER key pressed, for non-selected one �NEXT� key should be pressed)
				//	Select 2 start times
				//  Select Mode (Irrigation/ Fertigation)
				//	Select config for all valves if Irrigation mode selected
				//    	Check configured parameters  doesn�t have any overlapping
				//  Select Pre-wet time, config for all valves if Fertigation mode selected
				//    	Check configured parameters doesn�t have any overlapping
/*typedef enum
{
   PGM_A=1,  // PGM A  
   PGM_B, 	 // PGM B
}eProgram;
typedef enum
{
   PUMP_ON=1,  // Pump ON  
   PUMP_OFF, 	// Pump Off
}ePumpStatus;*/
typedef enum
{
	 RAIN_SHUT_OFF=0,   // INFO  If program stopped due to rain sensor signal
	 MAINS_OFF,        // ## MAINS_OFF=1 ,,INFO  When Mains power is not available.	
	 LOW_FLOW_2ND_RT, // ## LOW_FLOW_2ND_RT=2 ,, INFO  If Low or No Flow detected 2nd time in a row
	 LOW_BATTERY,     // ## LOW_BATTERY=3 INFO  ,, When battery is low
   PENDG_IRRG,			// ## PENDG_IRRG=4
	 PUMP_RECHARGING_TIME_ACTIVE, // ## PUMP_RECHARGING_TIME_ACTIVE=5 ,, INFO During Pump recharging period	
}eErrorInfoStatus;

/*
typedef enum
{
   IRRIGATION_ONLY_MODE=1, // Irrigation Mode 
   FERTIGATION_MODE   // Irrigation cum Fertigation Mode
}eIrrigationMode;*/

typedef struct
{
   //int valveDurV1Hrs;              // Valve V1 Duration Hours 
   //int valveDurV1Mins;             // Valve V1 Duration Mins 	
	int valveDurHrsV[MAX_VALVES];              // Valve V2 Duration Hours 
   int valveDurMinsV[MAX_VALVES];             // Valve V2 Duration Mins 
}stIrrigationModeConfig;

typedef struct
{
   int valveDurHrs;             // Valve Duration Hours 
   int valveDurMins;            // Valve Duration Mins
   int preWetDurHrs;            // Prewet Duration Hours 
   int preWetDurMins;           // Prewet Duration Mins
   int fertigationDurHrs;       // Fertigation Duration Hours 
   int fertigationDurMins;      // Fertigation Duration Mins	
   int flushingDurHrs;          // Flushing Duration Hours 
   int flushingDurMins;         // Flushing Duration Mins			 
}stFertgValveConfig;


typedef struct
{
    //stFertgValveConfig	FertgValveCfg[4]; //ori
	stFertgValveConfig	FertgValveCfg[12];	
}stFertigationModeConfig;


//check how to default initialize  
typedef struct
{
    //int diffStarttimeHrs;
    int diffStarttimeMins;
	  //int accValveDurHrs;
	  int accValveDurMins;	
	  //int accfertgDurHrs;
	  int accfertgDurMins;
    int StarttimeMins[2];	
}stlocalDerivedParams;


typedef struct
{
   //eIrrigationMode irrgMode;    // Irrigation or Fertigation Mode
	 int irrgMode;    // Irrigation or Fertigation Mode
	 int starttimeHrs[2];         // Start time Hrs
	 int starttimeMins[2];	      // Start time Mins
	 stlocalDerivedParams localdrivedParams;
	 union 
	 {
	    stIrrigationModeConfig  irrgCfg;
		  stFertigationModeConfig fertgCfg;
	 }irrgModeCfg;
}stIrrProgram;





void createNextSchedule(stIrrProgram *pIrrigationPgm);

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/*
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOA
#define VBAT_Pin GPIO_PIN_3
#define VBAT_GPIO_Port GPIOA
#define SW_6_Pin GPIO_PIN_4
#define SW_6_GPIO_Port GPIOA
#define SW_6_EXTI_IRQn EXTI4_15_IRQn
#define SW_5_Pin GPIO_PIN_5
#define SW_5_GPIO_Port GPIOA
#define SW_5_EXTI_IRQn EXTI4_15_IRQn
#define SW_4_Pin GPIO_PIN_6
#define SW_4_GPIO_Port GPIOA
#define SW_4_EXTI_IRQn EXTI4_15_IRQn
#define SW_3_Pin GPIO_PIN_7
#define SW_3_GPIO_Port GPIOA
#define SW_3_EXTI_IRQn EXTI4_15_IRQn
#define SW_2_Pin GPIO_PIN_0
#define SW_2_GPIO_Port GPIOB
#define SW_2_EXTI_IRQn EXTI0_1_IRQn
#define SW_1_Pin GPIO_PIN_1
#define SW_1_GPIO_Port GPIOB
#define SW_1_EXTI_IRQn EXTI0_1_IRQn
#define MAINS_SN_Pin GPIO_PIN_9
#define MAINS_SN_GPIO_Port GPIOA
#define SENSOR_2_Pin GPIO_PIN_11
#define SENSOR_2_GPIO_Port GPIOA
#define SENSOR_1_Pin GPIO_PIN_12
#define SENSOR_1_GPIO_Port GPIOA
#define LCD_VCC_Pin GPIO_PIN_8     // changed as per new board 
#define LCD_VCC_GPIO_Port GPIOA		// changed as per new board
#define LCD_RS_Pin GPIO_PIN_15
#define LCD_RS_GPIO_Port GPIOA
#define LCD_EN_Pin GPIO_PIN_3
#define LCD_EN_GPIO_Port GPIOB
#define LCD_D4_Pin GPIO_PIN_4
#define LCD_D4_GPIO_Port GPIOB
#define LCD_D5_Pin GPIO_PIN_5
#define LCD_D5_GPIO_Port GPIOB
#define LCD_D6_Pin GPIO_PIN_6
#define LCD_D6_GPIO_Port GPIOB
#define LCD_D7_Pin GPIO_PIN_7
#define LCD_D7_GPIO_Port GPIOB
#define LCD_LED_Pin GPIO_PIN_8
#define LCD_LED_GPIO_Port GPIOB
*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __CONFIG_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

