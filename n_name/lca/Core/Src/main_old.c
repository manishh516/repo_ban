/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "config.h"

#include "lcd_16x2.h"
#include "eeprom_AT24xxx.h"

#include "PCF8574.h" // ## @

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ec_200_lib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;



#define PUMP_ON_SET 0x80 // 0x02  // |   0x80
#define PUMP_OFF 0x7F	 // 0xFD  // &   0x7F

#define FERTG_ON_SET 0x20 // 0x40  // |    0x20
#define FERTG_OFF 0xDF	  // 0xBF  // &    0xDF

#define BP_ON_SET 0x40 // 0x20  // |    0x40
#define BP_OFF 0xBF	   // 0xDF  // &    0xBF

#define VALVE1_ON_SET 0x10 // 0x10//       0x10
#define VALVE2_ON_SET 0x08 // 0x20//       0x08
#define VALVE3_ON_SET 0x04 // 0x40//       0x04
#define VALVE4_ON_SET 0x02 // 0x80//       0x02
#define LCD_MAX_TIMEOUT 12

/* USER CODE BEGIN PV */

/* USER CODE END PV */


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
static void MX_IWDG_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);


/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

//uint8_t gnn[8]={0}; // @@ ## for irgg 
//uint8_t hnn[10]={0}; // @@ ## for fert
uint8_t pre_pcf_state = 0x00; // @@ ##
uint8_t shift_data=0; //## fn valve_output()
uint8_t pcf_dataa=0; // ## fn valve_output()
uint8_t pcf_return=0; // ##
uint8_t pcf_return_inbuilt=0;
//uint8_t off =0x00; // ##


/* USER CODE BEGIN PFP */
extern uint8_t pumpStatus;
extern uint8_t pumpInitiationTimeSecs;
extern uint16_t pumpRechargingTimeHrs;
extern uint16_t pumpRechargingTimeMins;
extern uint16_t pumpRechargingTimeAbsMins, pumpRechargingTimeCnt;
extern int Pgm;
// extern int  weekdays[8]; // 1 0 0 0 0 0 0 0 0  (all days)
//  0 x x x x x x x x   (select days)
extern uint8_t weekdays;
extern uint8_t NextSchedDay, weekdaysPgmA, weekdaysPgmB;
extern uint8_t NextSchedDayPgmA, NextSchedDayPgmB;
extern stIrrProgram IrrigationPgm[2]; // Program A and B
									  // ePumpStatus  pumpStatus;
extern eErrorInfoStatus errorInfo[8];
extern uint8_t errorFlag;
// extern int  errorFlag[8];

// int pump_status[2] = {PUMP_ON, PUMP_OFF};
// int pgm[2] = {PGM_A, PGM_B};

extern uint8_t pump_status[2];
extern int pgm_mode[5];
extern uint8_t mode;
extern int pump_pgm[3];
extern int pgm[2];
// eIrrigationMode irrgModeSet[2]={IRRIGATION_ONLY_MODE, FERTIGATION_MODE};
// int irrgModeSet[2]={FERTIGATION_MODE, IRRIGATION_ONLY_MODE };
extern int irrgModeSet[2];
extern char errorInfoArr[2][17];
// eIrrigationMode IrrigationMode = IRRIGATION_ONLY_MODE;
// extern char status[2][3];
extern char choice[2][4];
extern char week[7][10];
extern char months[12][10];

extern uint16_t set_DurHrs[24];
extern uint16_t set_DurMins[12];
extern int set_weekdays[2];
// extern int set_weekdaysPgmA[8];
// extern int set_weekdaysPgmB[8];
extern uint16_t set_dpdelay[5];
extern int set_DurSecs[];
extern char lcd_data[100];
// uint8_t interval,duration,dp_delay;
// uint8_t interval_1,duration_1,dp_delay_1;
extern int dp_sw_status, dp_only, manual_sw_status, loop_flag, flush_flag, duration_tick, dip_duration, interval_tick, run_mode, conf_flag;
// extern int duration_1min;
extern char msg[40];
extern volatile uint16_t sw1, sw2, sw3, swbk;
extern uint16_t swtemp;
extern int durHrs, durMins, cnfgDn, pgmcnfgDn;
extern int MenuDoneCnt, backflag, mains_off_flag;
extern int mode_flag;

extern uint8_t date[31];
extern uint8_t month[12];

extern uint8_t year[7];
extern uint8_t mins[60];
extern uint8_t day[7];
extern uint8_t d_t;
extern uint8_t index1;

extern int get_date, get_month, get_year, get_hrs, get_mins, get_day;

extern uint8_t irrigationMode[2];
extern uint8_t pumpstate;
extern uint16_t masterSchdPumpSttimeMins[4];
extern uint16_t masterSchdValveSttimeMins[48];
extern uint16_t masterSchdValveDurationMins[48];

extern uint16_t masterSchdPreWetSttimeMins[48];
extern uint16_t masterSchdFertgSttimeMins[48];
extern uint16_t masterSchdFlushingSttimeMins[48];

extern uint16_t masterSchdPreWetDurMins[48];
extern uint16_t masterSchdFertgDurMins[48];
extern uint16_t masterSchdFlushingDurMins[48];

extern uint16_t todaySchdPumpSttimeMins[4];		//{0xffff,0xffff,0xffff,0xffff};
extern uint16_t todaySchdValveSttimeMins[48];	//{0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff};
extern uint16_t todaySchdValveDurationMins[48]; //{0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff};

extern uint16_t todaySchdPreWetSttimeMins[48];	 //{0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff};
extern uint16_t todaySchdFertgSttimeMins[48];	 //{0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff};
extern uint16_t todaySchdFlushingSttimeMins[48]; //{0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff};

extern uint16_t todaySchdPreWetDurMins[48];	  //{0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff};
extern uint16_t todaySchdFertgDurMins[48];	  //{0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff};
extern uint16_t todaySchdFlushingDurMins[48]; //{0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff};

extern uint8_t valve_count, pgm_A_set, pgm_B_set;

extern char pgm_mode_disp[8][14];

extern uint8_t get_total_v; // @@ ## total val no.
uint8_t displayConfig_flag =0;

// EEPROM TEST
uint8_t errorFlag1;
uint8_t pumpStatus1;			 //=1;
uint8_t pumpInitiationTimeSecs1; //=90;
uint16_t pumpRechargingTimeAbsMins1;
uint8_t weekdays1;
uint8_t irrigationMode1[2];
uint8_t CurrSchedIdx1;
uint16_t LastTimeInstanceMins1;
uint16_t masterSchdPumpSttimeMins1[4];
uint16_t masterSchdValveSttimeMins1[48];
uint16_t todaySchdPumpSttimeMins1[4];
uint16_t todaySchdValveSttimeMins1[48];
uint16_t todaySchdValveDurationMins1[48];
uint8_t wait_flag = 1;

//// pub_msg_sch ##
//uint8_t live_msg_flg=0;
//uint8_t status_msg_flg=0;
//uint8_t status_chg_flg=0;
//uint8_t curr_v=0;
//uint8_t pre_curr_v=0;
//uint8_t fert_chg_flg=0;
//uint8_t pre_pump_state;


extern uint8_t get_exp; // ##
//extern uint8_t v_i_12[13]; // ##
//extern char v_c_12[13][14]; // ##
//extern uint8_t gett_v;	// ##

//text_mode() fn vars ##
//uint8_t rep_i[4] = {1, 2, 30, 50}; // ##
//char rep_c[4][3] = {"1", "2", "30", "50"}; // ##
uint8_t rep_v=0;	  // ##
uint8_t rep_stop=0; // ##


extern uint8_t CurrSchedIdx;
extern uint8_t NextSchedIdx;
extern uint8_t FirstSchedIdxPgmA, FirstSchedIdxPgmB;
extern uint8_t LastSchedIdxPgmA, LastSchedIdxPgmB;

extern uint16_t CurrSchdlValve_StarttimeMins;
// extern uint16_t CurrSchdlValve_RunDurationMins;
// extern uint16_t CurrSchdlValve_RunningStatus;

// extern uint16_t NextSchdlValve_StarttimeMins;
// extern uint16_t NextSchdlValve_RunDurationMins;
// extern uint16_t NextSchdlValve_RunningStatus;

// used to restart interuppted operation
extern uint16_t LastTimeInstanceMins;
extern uint16_t currRTCtimeMins;
extern uint16_t SecondRTTimeInstanceMins;
extern uint8_t dayIdx;
extern uint8_t SecondRTdayIdx;

extern uint16_t Ignore_flag_PInit_Rechg0;
extern uint16_t startup_Flag; // to be set at each start time and to reset just after it
extern uint16_t running_Flag; // this flag to remain set till all valves have not completed
extern uint16_t updated_flag;
extern uint16_t Run_State;
extern uint16_t scheduleToBeUpdated_Flag;
extern uint16_t Specialstate;
extern uint8_t ScheduleComplete, prevWeekDay;
extern uint16_t statePrewetSetDone, stateFertigationSetDone, stateFlushingSetDone;
//extern uint16_t minsFlag;
uint8_t runBalflag = 1;

// Flags
extern int Rain_Signal_Flag;
extern int Low_No_Flow_Signal_Flag;
extern int mains_off_Signal_Flag;
extern int LCD_ON_Flag;
extern int POWER_ON_Flag;
extern int SpecialEv_ON_Flag;
extern int SpecialEv_OFF_Flag;
extern int PUMP_STATUS_Flag;
extern int prewet_flagA, prewet_flagB;
extern int flushing_flagA, flushing_flagB;
extern int fertigation_flagA, fertigation_flagB;
extern int fertigationValveOut, bypassValveOut;
extern int low_no_flow_count;
extern int halt_alloperation;

extern int validateStartTime(stIrrProgram *pIrrigationPgm);
extern int checkValveConfig(stIrrProgram *pIrrigationPgm);
extern void addtime(int *SumH, int *SumM, int *BH, int *BM);
extern int checkFertgConfig(int AM, int BM);
extern void createMasterSchedule(stIrrProgram *pIrrigationPgm);
extern void createTodaySchedule(stIrrProgram *pIrrigationPgm);
extern void pump_setting(void);
extern void parameter_config(void);

void RTC_TimeShow(void);
void RTC_TimeShow1(void);
void RTC_TimeShow_get(void);

void uct_EEPROM_READ(void);
void uct_EEPROM_FIXWRITE(int);
void uct_EEPROM_UPDATEDWRITE(void);
void printPgmStatus(void);
void displayConfig(void);
void lcd_on(void);
void lcd_off(void);
void valve_output(uint8_t *data, uint8_t *pcf_data); // ##

void valve_output_test(uint8_t *data, uint8_t *pcf_data);

// int checkFertgConfig( int AH, int AM, int BH, int BM);
// execution
extern uint8_t mains_status, rain_status, low_flow_status; // ch-type
extern uint8_t mains_status1, rain_status1, low_flow_status1; // ch-type
extern RTC_TimeTypeDef sTime, gTime;
extern uint32_t current_time;
extern uint32_t schedule_time[4];
extern uint8_t start_irrigation_flag[4];
extern uint8_t pump_initiation, busy, recharge_flag;
extern uint8_t schedule_1[12], schedule_2[12], schedule_3[12], schedule_4[12], pump_init_time;
extern uint16_t recharge_time;

extern int statusFlag;
extern uint16_t menu, value, looping, sw1_max, sw2_max, swbk_max, input_timeout, lock, interval, duration, dp_value, dp_delay;
extern uint16_t lcd_retain_flag1;
extern uint16_t lcd_retain_flag2;
extern uint8_t running, nextDayOverlapFlag, pendIrrgFlag;
uint8_t running_pcf; // ##
extern uint8_t effectiveFromToday;
uint8_t gvCommand = 0, k = 0X00, disp = 0, disp_cnfg = 0;
uint8_t gvCommand_pcf = 0; // ##

int dbgCnt = 0;
int pumpinitCnt = 0;
int count = 1;
int lcd_on_flag = 0;
extern int lcd_timeout;
uint8_t notconfigured = 0; // ch-type
int daynotconfigured = 0;
char pgm_nmnc[2][2] = {"A", "B"};
int test_mode_flag = 0;
uint8_t Balprint = 1; // ## ch-ty
uint16_t balHrs = 0, balMins = 0; // ## ch-ty
uint16_t balMins_msg=0; // ## 
//uint16_t balHrs1 = 0, balMins1 = 0; // ## ch-ty
// updated by rishabh
uint8_t pump_start_status = 0;
uint16_t minscount = 0;
int checkOnce = 1;
int day_reset = 0, reset_counter = 0;
char apn[]= "www";
char server[]="34.146.195.77";
char port[]="1883";
char topic[]="test_1";
char msg_up[]="testing boot msg";
extern char send_msg[];


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// 5sec timer for IO status
	if (htim->Instance == TIM6)
	{
		// press both sw4 and sw5 for reset after 2nd low flow error  ## 
		if ((errorFlag & (1 << LOW_FLOW_2ND_RT)) == 0)
		{
			if ((HAL_GPIO_ReadPin(SW_4_GPIO_Port, SW_4_Pin) == 0) && (HAL_GPIO_ReadPin(SW_5_GPIO_Port, SW_5_Pin) == 0))
			{
				HAL_NVIC_SystemReset();
			}
		}

		duration_tick++;
		lcd_timeout++;


		// input_timeout++;
		// HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		mains_status = HAL_GPIO_ReadPin(MAINS_SN_GPIO_Port, MAINS_SN_Pin);

		// new order
		if (lcd_timeout > LCD_MAX_TIMEOUT && mains_status == 0 && lcd_on_flag == 1 && test_mode_flag == 0)
		{
			lcd_off();
		}


		rain_status1 = HAL_GPIO_ReadPin(RAIN_SENSOR_GPIO_Port, RAIN_SENSOR_Pin);
		rain_status = !rain_status1;


		if ((pump_start_status == 1) && (pump_initiation == 0) && (halt_alloperation != 1) && (rain_status == 0)) // && (pumpRechargingTimeAbsMins !=0))
		{
			low_flow_status1 = HAL_GPIO_ReadPin(LOW_FLOW_GPIO_Port, LOW_FLOW_Pin);

			// if (low_flow_status1 == 1)
			// {
			//		halt_alloperation =0;
			// }
			// low_flow_status = !low_flow_status1;
			low_flow_status = low_flow_status1;
		}

		// when rain error occur ##
		if ((rain_status == 1) && (Rain_Signal_Flag == 0))
		{
			// stop irrigation
			Specialstate = ON;
			SpecialEv_ON_Flag = 1;
			Balprint = 1;
			wait_flag = 0;  // if 1 then show NEXT_sch going to run message on lcd because there is no error ##
			Rain_Signal_Flag = 1;
			scheduleToBeUpdated_Flag = 1;
			// save timing in EEPROM
			LastTimeInstanceMins = currRTCtimeMins;
		}


		// when low flow error occur ##
		if ((low_flow_status == 1) && (Low_No_Flow_Signal_Flag == 0) && (pump_initiation == 0))
		{
			// stop irrigation
			// go in pump initiation

			Specialstate = ON;
			SpecialEv_ON_Flag = 1;
			Balprint = 1;
			wait_flag = 0;
			Low_No_Flow_Signal_Flag = 1;
			pumpRechargingTimeCnt = 0;
			scheduleToBeUpdated_Flag = 1;
			// save timing in EEPROM
			LastTimeInstanceMins = currRTCtimeMins;
		}
		
		// ## error , mains_status = 0 , means mains_off  
		if ((mains_status == 0) && (mains_off_Signal_Flag == 0))
		{
			// stop irrigation
			// save progress of the current program

			Specialstate = ON;
			SpecialEv_ON_Flag = 1;
			Balprint = 1;
			wait_flag = 0;
			mains_off_Signal_Flag = 1;
			pump_initiation = 1;
			scheduleToBeUpdated_Flag = 1;
			Low_No_Flow_Signal_Flag = 0;
			// save timing in EEPROM
			if (Run_State == 1)
				LastTimeInstanceMins = currRTCtimeMins;
		}

		// ## when rain error correct/stop, pcf frc also added
		if ((rain_status == 0) && (Low_No_Flow_Signal_Flag == 0) && ((mains_status == 1)) && (disp == 0))
		{
			uint8_t tempFlag = 0XFE; //!(1<<RAIN_SHUT_OFF);
			errorFlag &= tempFlag;
			Specialstate = OFF;
			SpecialEv_OFF_Flag = 1;
			Rain_Signal_Flag = 0;
			// start irrigation
			// start pump
		}

		// ## when low flow error correct
		if ((low_flow_status == 0) && (Rain_Signal_Flag == 0) && (mains_status == 1) && (disp == 0))
		{
			Specialstate = OFF;
			SpecialEv_OFF_Flag = 1;
			Low_No_Flow_Signal_Flag = 0;
			low_no_flow_count = 0;
			// start irrigation
			// go in pump initiation
		}

		// ## when main_on error correct
		if ((mains_status == 1) && (Rain_Signal_Flag == 0) && (Low_No_Flow_Signal_Flag == 0) && (disp == 0))
		{
			uint8_t tempFlag = 0xFD; //!(1<<MAINS_OFF);
			Specialstate = OFF;
			SpecialEv_OFF_Flag = 1;
			mains_off_Signal_Flag = 0;
			errorFlag &= tempFlag;
			// start irrigation
			// save progress of the current program
		}
	}
	// 1min timer (needed for flushing interval)

	if (htim->Instance == TIM14)
	{
		// used for checking shchedule
		//		HAL_RTC_GetTime(&hrtc,&gTime,RTC_FORMAT_BIN);
		//		current_time=gTime.Hours*60+gTime.Minutes;
		currRTCtimeMins++; // = current_time;
		minscount++;

		// added by rishabh
		if (day_reset == 1)
		{
			// ## previous value of reset_counter=0
			reset_counter++;
		}
		// ## in think, after whole 5 days sch complete , device reset
		if (reset_counter >= 5)
		{
			
			NVIC_SystemReset();
		}

		// ## not understand --> is when pg not run then passed minscount=60 --> its value change to 0
		if ((Run_State == 0) && (masterSchdValveSttimeMins[CurrSchedIdx] > 0) && (minscount >= 60))
		{
			RTC_TimeShow_get();
			minscount = 0;
		}

		
		// when day's 24 hours complete and passed that ##
		if (currRTCtimeMins >= 1440)
		{
			// ## when whole day sch complete and so , nextDayOverlapFlag =0 
			if (nextDayOverlapFlag == 0)
			{

				if (Run_State == 0 && pendIrrgFlag == 0 && recharge_flag == 0) // changed by rishabh
				{
					// ## when whole day sch complete --> this increment reset_counter by 1 then at (reset_counter >= 5) --> device reset
					day_reset = 1; // changed by rishabh
				}
				
				dayIdx++;  // after whole day sch complete --> it increment 

				// ## this dayIdx value range from 1 to 8, at 8 --> its value change to 1 
				if (dayIdx > 7)
				{
					dayIdx = 1;
				}

				if (prevWeekDay != dayIdx)
				{
					prevWeekDay = dayIdx;
				}
				// HAL_NVIC_SystemReset();
				//					RTC_TimeShow();
				currRTCtimeMins = 0;
				if (effectiveFromToday == 0)
				{
					uint8_t *Data;
					effectiveFromToday = 1;
					Data = (uint8_t *)&effectiveFromToday;
					EEPROM_WriteNBytes(EP_EFFECTIVETODAY, Data, 1); // Valve count
				}
				if (((weekdays & 0x01) == 1) || (((weekdays >> dayIdx) & 0x01) == 1))
				{
					uint8_t *Data;

					ScheduleComplete = 0;
					Data = &ScheduleComplete;
					EEPROM_WriteNBytes(EP_SCHEDULE_COMPLETE, Data, 1); // Error Flag
				}
				createTodaySchedule(&IrrigationPgm[0]);

				// change by sunny
				uint8_t *Data;
				pendIrrgFlag = 0;
				Data = (uint8_t *)&pendIrrgFlag;
				EEPROM_WriteNBytes(EP_PENDIRRG, Data, 1); // pendIrrgFlag Flag
			}

			// ## when whole day sch not-complete and so , nextDayOverlapFlag =1, that why pendIrrgFlag = 1 , value stored in eprom
			else
			{
				// change by sunny
				uint8_t *Data;

				pendIrrgFlag = 1;
				Data = (uint8_t *)&pendIrrgFlag;
				EEPROM_WriteNBytes(EP_PENDIRRG, Data, 1); // pendIrrgFlag Flag
			}
		}

		// when low flow error correct ##
		if (Low_No_Flow_Signal_Flag == 1)
			pumpRechargingTimeCnt++;  // not understand --> but this is related only to low flow error##



	}
}



void startValve(int CurrValveIdx)
{
	uint8_t vstart_command;
	uint8_t vstart_command_pcf;
	
	if ((get_exp ==0 || get_exp==1) && CurrValveIdx <=4)
	{
		//uint8_t vstart_command; // ##
		// vstart_command = 1<< (CurrValveIdx + 3);
		vstart_command = 1 << (5 - CurrValveIdx);

		gvCommand = gvCommand | vstart_command;
		gvCommand = gvCommand | BP_ON_SET;
		gvCommand = gvCommand & FERTG_OFF;
		// valve_output(&k);
//		gnn[1] = gvCommand; // ## for irgg
//		hnn[1] = gvCommand; // ## for fert
		
		gvCommand_pcf = 0x00;

		valve_output(&gvCommand,&gvCommand_pcf);
	}

	if (get_exp ==1 && CurrValveIdx >=5)
	{
		
		vstart_command =0;
		
		vstart_command_pcf = 1 << ((5 - CurrValveIdx)*-1);
		
		//k=1 << ((5 - y)*-1);

		gvCommand = gvCommand | vstart_command;
		gvCommand = gvCommand | BP_ON_SET; //0x40 ##
		gvCommand = gvCommand & FERTG_OFF; //0x40 ##
	
		gvCommand_pcf = vstart_command_pcf;
		
//		gnn[2] = gvCommand_pcf; // ##
//		hnn[2] = gvCommand_pcf; // ##
		valve_output(&gvCommand,&gvCommand_pcf);
	}


}

// Make sure currently enabled valve is on rest Solenoid valve are reset
void stopValve(int CurrValveIdx)
{

	if ((get_exp ==0 || get_exp ==1) && CurrValveIdx <=4)
	{
		if (CurrValveIdx == 2)
		{
			gvCommand = 0xC8;
			// gvCommand = gvCommand & (!vstart_command);
		}
		else if (CurrValveIdx == 3)
		{
			gvCommand = 0xC4;
			// gvCommand = gvCommand & (!vstart_command);
		}
		else if (CurrValveIdx == 4)
		{
			gvCommand = 0xC2;
			// gvCommand = gvCommand & (!vstart_command);
		}
		else
		{
			gvCommand = 0x0;
			// gvCommand = gvCommand & (!vstart_command);
		}

//		gnn[9]=gvCommand; // ##
//		hnn[9]=gvCommand; // ##
		
		gvCommand_pcf = 0x00;
		valve_output(&gvCommand,&gvCommand_pcf);
	}
	
	if (get_exp ==1 && CurrValveIdx >=5)
	{

		if (CurrValveIdx == 5)
		{
			gvCommand = 0xC0; // only bypass and pump run --> for shift reg
			gvCommand_pcf = 0x01;	
		}
		
		else if (CurrValveIdx == 6)
		{
			gvCommand = 0xC0; // only bypass and pump run --> for shift reg
			gvCommand_pcf = 0x02;	
		}
		
		else if (CurrValveIdx == 7)
		{
			gvCommand = 0xC0; // only bypass and pump run --> for shift reg
			gvCommand_pcf = 0x04;	
		}
		
		else if (CurrValveIdx == 8)
		{
			gvCommand = 0xC0; // only bypass and pump run --> for shift reg
			gvCommand_pcf = 0x08;	
		}
		
		else if (CurrValveIdx == 9)
		{
			gvCommand = 0xC0; // only bypass and pump run --> for shift reg
			gvCommand_pcf = 0x10;	
		}
	
		else if (CurrValveIdx == 10)
		{
			gvCommand = 0xC0; // only bypass and pump run --> for shift reg
			gvCommand_pcf = 0x20;	
		}
		
		else if (CurrValveIdx == 11)
		{
			gvCommand = 0xC0; // only bypass and pump run --> for shift reg
			gvCommand_pcf = 0x40;	
		}
		
		else if (CurrValveIdx == 12)
		{
			gvCommand = 0xC0; // only bypass and pump run --> for shift reg
			gvCommand_pcf = 0x80;	
		}
		else
		{
			gvCommand = 0x00; // bypass ;pump; all_valve off --> for shift reg
			gvCommand_pcf = 0x00;
			
		}
	
//		gnn[10] = gvCommand; // ##
//		gnn[11] = gvCommand_pcf; // ##
//		hnn[10] = gvCommand; // ##
//		hnn[11] = gvCommand_pcf; // ##
		valve_output(&gvCommand,&gvCommand_pcf);
	
	
	}
}

// i think, startPump()--> only run at pump init time or v1 run time(0xD0) --> after that stopValve() take care of pump runing
//if_only_pump_and_bp_run_and_valve_off_1_to_4=0xC0, pump_and_bp_run_v2_on=0xC8, pump_and_bp_run_v3_on=0xC4, pump_and_bp_run_v4_on=0xC2 --> only of irgg mode
void startPump()
{
	
	if (get_exp ==0)
	{
		gvCommand = gvCommand | PUMP_ON_SET;
		
//		gnn[3] = gvCommand; // ##
//		hnn[3] = gvCommand; // ##
		
		gvCommand_pcf = 0x00;
		
		//for on
		HAL_GPIO_WritePin(RELAY_PWR_GPIO_Port,RELAY_PWR_Pin,0);
		HAL_GPIO_WritePin(PUMP_R_GPIO_Port,PUMP_R_Pin,1);
		
		valve_output(&gvCommand,&gvCommand_pcf);
		pump_start_status = 1;

	}

	if (get_exp ==1)
	{
		gvCommand = gvCommand | PUMP_ON_SET;
//		gnn[4] = gvCommand; // ## 0xC0
//		gnn[5] = gvCommand_pcf; // ##
//		
//		hnn[4] = gvCommand; // ## 
//		hnn[5] = gvCommand_pcf; // ##
		
		//for on
		HAL_GPIO_WritePin(RELAY_PWR_GPIO_Port,RELAY_PWR_Pin,0);
		HAL_GPIO_WritePin(PUMP_R_GPIO_Port,PUMP_R_Pin,1);
		
		valve_output(&gvCommand,&gvCommand_pcf);
		pump_start_status = 1;

	}

}

void stopPump()
{
	if (get_exp ==0)
	{
		gvCommand = gvCommand & PUMP_OFF;
		//	valve_output(&k);
//		gnn[6] = gvCommand; // ##
//		hnn[6] = gvCommand; // ##
		
		gvCommand_pcf = 0x00;
		valve_output(&gvCommand,&gvCommand_pcf);
		
		//for off
		HAL_GPIO_WritePin(RELAY_PWR_GPIO_Port,RELAY_PWR_Pin,1);
		HAL_GPIO_WritePin(PUMP_R_GPIO_Port,PUMP_R_Pin,0);
		
		pump_start_status = 0;
	}

	if (get_exp ==1)
	{
		gvCommand = gvCommand & PUMP_OFF;
		//	valve_output(&k);
//		gnn[7] = gvCommand; // ##
//		gnn[8] = gvCommand_pcf; // ##
//		hnn[7] = gvCommand; // ##
//		hnn[8] = gvCommand_pcf; // ##
		
		//for off
		HAL_GPIO_WritePin(RELAY_PWR_GPIO_Port,RELAY_PWR_Pin,1);
		HAL_GPIO_WritePin(PUMP_R_GPIO_Port,PUMP_R_Pin,0);
		
		valve_output(&gvCommand,&gvCommand_pcf);
		pump_start_status = 0;
	}

}

// ## this run in fert-pg-mode
// ## here CurrValveIdx = CurrSchedIdx ,, not plus one , like CurrValveIdx =vidx+1 in startValve(CurrValveIdx)
//void startPrewetByPass(int CurrValveIdx) // ## ori
void startPrewetByPass(int CurrSchedIdx_x)    // ## here CurrSchedIdx_x = CurrSchedIdx, --> this requied because fert-pg-mode not run good after v4 for st2, shift_data=good--pcf_data=0=bad(for_all_val)
{
	uint8_t i, u;
	uint8_t vstart_command_pcf=0; // ##

	uint8_t vidx = CurrSchedIdx_x % 12;
	uint8_t CurrValveIdx_fert = vidx+1; // ## for v1 --> vidx=0

	if ((get_exp ==0 || get_exp==1) && CurrValveIdx_fert <=4)
	{
		gvCommand = gvCommand | BP_ON_SET;
		gvCommand = gvCommand & FERTG_OFF;
		
//		hnn[12]=gvCommand; // ##
		
		gvCommand_pcf=0x00; // ##
		valve_output(&gvCommand,&gvCommand_pcf); // fert-@@ --temp-modify,, chg it--

	}

	if (get_exp ==1 && CurrValveIdx_fert >=5)
	{

		//gvCommand value start with 0x80
		gvCommand = gvCommand | BP_ON_SET;
		gvCommand = gvCommand & FERTG_OFF; // 0xC0

		//uint8_t vstart_command_pcf; // del not work ##
		vstart_command_pcf = 1 << ((5 - CurrValveIdx_fert)*-1); // del not work ##
		gvCommand_pcf = vstart_command_pcf; // del not work ##
		
//		hnn[13]=gvCommand; // ##
//		hnn[14]=gvCommand_pcf; // ##
		valve_output(&gvCommand,&gvCommand_pcf); // fert-@@ --temp-modify,, chg it--
	}

	if ((CurrSchedIdx < 12) && (CurrSchedIdx >= 0))
	{
		i = 0;
		u = 1;
	}
	if ((CurrSchedIdx < 24) && (CurrSchedIdx >= 12))
	{
		i = 0;
		u = 2;
	}
	if ((CurrSchedIdx < 36) && (CurrSchedIdx >= 24))
	{
		i = 1;
		u = 1;
	}
	if ((CurrSchedIdx < 48) && (CurrSchedIdx >= 36))
	{
		i = 1;
		u = 2;
	}

	LCD_Clear();
	LCD_Gotoxy(0, 0);
	sprintf(msg, "PGM-%s ST%d ", pgm_nmnc[i], u);
	LCD_Puts(msg);
	LCD_Gotoxy(0, 1);
	//sprintf(msg, "V%d ON PREWET ", (CurrValveIdx % 12) + 1);
	sprintf(msg, "V%d ON PREWET ", CurrValveIdx_fert);
	LCD_Puts(msg);

	//		sprintf(msg,"V%d ON    ", (CurrValveIdx&3)+1);
	//		LCD_Puts(msg);

	//		LCD_Gotoxy(0,1);
	//		LCD_Puts("PREWET");
}



// @@ this run in fert mode
void startFertigation(int CurrSchedIdx_x)
{
	uint8_t i, u;
	uint8_t vstart_command_pcf=0; // ##
	
	uint8_t vidx = CurrSchedIdx_x % 12;
	uint8_t CurrValveIdx_fert = vidx+1; // ## for v1 --> vidx=0
	
	
	if ((get_exp ==0 || get_exp==1) && CurrValveIdx_fert <=4)
	{
		
		gvCommand = gvCommand | FERTG_ON_SET;
		gvCommand = gvCommand & BP_OFF;
//		hnn[15]=gvCommand; // ##
		
		gvCommand_pcf=0x00; // ##
		
		valve_output(&gvCommand,&gvCommand_pcf); // fert-@@
		
	}

	if (get_exp ==1 && CurrValveIdx_fert >=5)
	{
		
		//gvCommand value start with 0x80
		gvCommand = gvCommand | FERTG_ON_SET;
		gvCommand = gvCommand & BP_OFF; // 0xA0

		//uint8_t vstart_command_pcf; // del not work ##
		vstart_command_pcf = 1 << ((5 - CurrValveIdx_fert)*-1); // del not work ##
		gvCommand_pcf = vstart_command_pcf; // del not work ##
		
//		hnn[16]=gvCommand; // ##
//		hnn[17]=gvCommand_pcf; // ##
		
		valve_output(&gvCommand,&gvCommand_pcf); // fert-@@
		
	}
	
	
	
	
	
	
	if ((CurrSchedIdx < 12) && (CurrSchedIdx >= 0))
	{
		i = 0;
		u = 1;
	}
	if ((CurrSchedIdx < 24) && (CurrSchedIdx >= 12))
	{
		i = 0;
		u = 2;
	}
	if ((CurrSchedIdx < 36) && (CurrSchedIdx >= 24))
	{
		i = 1;
		u = 1;
	}
	if ((CurrSchedIdx < 48) && (CurrSchedIdx >= 36))
	{
		i = 1;
		u = 2;
	}

	LCD_Clear();
	LCD_Gotoxy(0, 0);
	sprintf(msg, "PGM-%s ST%d ", pgm_nmnc[i], u);
	LCD_Puts(msg);
	LCD_Gotoxy(0, 1);
	//sprintf(msg, "V%d ON FERTGATION", (CurrValveIdx % 12) + 1);
	sprintf(msg, "V%d ON FERTGATION", CurrValveIdx_fert);
	LCD_Puts(msg);

	//	  LCD_Clear();
	//		LCD_Gotoxy(0,0);
	//		sprintf(msg,"V%d ON    ", (CurrValveIdx&3)+1);
	//		LCD_Puts(msg);
	//
	//		LCD_Gotoxy(0,1);
	//		LCD_Puts("FERTIGATION ");
}



// @@ this run in fert mode and here no valve , v1 , v2 etc.. run
void startFlushByPass(int CurrSchedIdx_x)
{
	uint8_t i, u;
	uint8_t vstart_command_pcf=0; // del not work ##
	uint8_t vidx = CurrSchedIdx_x % 12;
	
	uint8_t CurrValveIdx_fert = vidx+1;
	
	if ((get_exp ==0 || get_exp==1) && CurrValveIdx_fert <=4)
	{
		gvCommand = gvCommand | BP_ON_SET;
		gvCommand = gvCommand & FERTG_OFF;
		
//		hnn[18]=gvCommand; // ##
		
		gvCommand_pcf=0x00; // ##
		valve_output(&gvCommand,&gvCommand_pcf); // fert-@@ --temp-modify,, chg it--

	}

	if (get_exp ==1 && CurrValveIdx_fert >=5)
	{
		//gvCommand value start with 0x80
		gvCommand = gvCommand | BP_ON_SET;
		gvCommand = gvCommand & FERTG_OFF; // 0xC0

		//uint8_t vstart_command_pcf; // del not work ##
		vstart_command_pcf = 1 << ((5 - CurrValveIdx_fert)*-1); // del not work ##
		gvCommand_pcf = vstart_command_pcf; // del not work ##
		
//		hnn[19]=gvCommand; // ##
//		hnn[20]=gvCommand_pcf; // ##
		
		valve_output(&gvCommand,&gvCommand_pcf); // fert-@@ --temp-modify,, chg it--
	}
	
	
	
	
	
	if ((CurrSchedIdx < 12) && (CurrSchedIdx >= 0))
	{
		i = 0;
		u = 1;
	}
	if ((CurrSchedIdx < 24) && (CurrSchedIdx >= 12))
	{
		i = 0;
		u = 2;
	}
	if ((CurrSchedIdx < 36) && (CurrSchedIdx >= 24))
	{
		i = 1;
		u = 1;
	}
	if ((CurrSchedIdx < 48) && (CurrSchedIdx >= 36))
	{
		i = 1;
		u = 2;
	}

	LCD_Clear();
	LCD_Gotoxy(0, 0);
	sprintf(msg, "PGM-%s ST%d ", pgm_nmnc[i], u);
	LCD_Puts(msg);
	LCD_Gotoxy(0, 1);
	//sprintf(msg, "V%d ON FLUSHING ", (CurrValveIdx % 12) + 1);
	sprintf(msg, "V%d ON FLUSHING ", CurrValveIdx_fert);
	LCD_Puts(msg);

	//		sprintf(msg,"V%d ON    ", (CurrValveIdx&3)+1);
	//		LCD_Puts(msg);
	//
	//		LCD_Gotoxy(0,1);
	//		LCD_Puts("FLUSHING");
}



void startSequence()
{
	uint8_t vidx = CurrSchedIdx % 12;
	uint8_t i, u;
	// Start the valve Vn
	startValve(vidx + 1);
	LCD_Clear();
	LCD_Gotoxy(0, 1);
	sprintf(msg, "VALVE%d STARTED", vidx + 1);
	LCD_Puts(msg);
	// Sleep(100);  //how much delay

	// Start pump;
	startPump();
	// LCD_Gotoxy(0,0);
	// LCD_Puts("Pump Started...   ");
	if (pumpInitiationTimeSecs != 0)
		pump_initialization();
	else
		pump_initiation = 0;

	if ((CurrSchedIdx < 12) && (CurrSchedIdx >= 0))
	{
		i = 0;
		u = 1;
	}
	if ((CurrSchedIdx < 24) && (CurrSchedIdx >= 12))
	{
		i = 0;
		u = 2;
	}
	if ((CurrSchedIdx < 36) && (CurrSchedIdx >= 24))
	{
		i = 1;
		u = 1;
	}
	if ((CurrSchedIdx < 48) && (CurrSchedIdx >= 36))
	{
		i = 1;
		u = 2;
	}

	LCD_Clear();
	LCD_Gotoxy(0, 0);
	sprintf(msg, "PGM-%s ", pgm_nmnc[i]);
	LCD_Puts(msg);
	LCD_Gotoxy(0, 1);
	sprintf(msg, "ST%d V%d ON    %3s ", u, vidx + 1, week[dayIdx - 1]);
	LCD_Puts(msg);
}

void startSequenceTestMode()
{
	uint8_t vidx = 0;
	// Start the valve Vn
	startValve(vidx + 1);
	//		LCD_Clear();
	//		LCD_Gotoxy(0,1);
	//		sprintf(msg,"VALVE%d STARTED", vidx+1);
	//		LCD_Puts(msg);
	// Sleep(100);  //how much delay

	// Start pump;
	startPump();
	// LCD_Gotoxy(0,0);
	// LCD_Puts("Pump Started...   ");
	pumpInitiationTimeSecs = 5;
	pump_initialization();

	//		LCD_Clear();
	//		LCD_Gotoxy(0,0);
	//		LCD_Puts("PUMP ON   ");
	//		LCD_Gotoxy(0,1);
	//		sprintf(msg,"V%d ON    ", vidx+1);
	//		LCD_Puts(msg);
}
#if 1
void updateValveSequence()
{
	uint8_t vidx = NextSchedIdx % 12;
	uint8_t i, u;

	if ((CurrSchedIdx < 12) && (CurrSchedIdx >= 0))
	{
		i = 0;
		u = 1;
	}
	if ((CurrSchedIdx < 24) && (CurrSchedIdx >= 12))
	{
		i = 0;
		u = 2;
	}
	if ((CurrSchedIdx < 36) && (CurrSchedIdx >= 24))
	{
		i = 1;
		u = 1;
	}
	if ((CurrSchedIdx < 48) && (CurrSchedIdx >= 36))
	{
		i = 1;
		u = 2;
	}

	LCD_Clear();
	LCD_Gotoxy(0, 0);
	sprintf(msg, "PGM-%s ", pgm_nmnc[i]);
	LCD_Puts(msg);
	LCD_Gotoxy(0, 1);
	sprintf(msg, "ST%d V%d ON    %3s ", u, vidx + 1, week[dayIdx - 1]);
	LCD_Puts(msg);

	// Start next scheduled valve Vn+1
	startValve(vidx + 1);

	// delay 2 secs
	HAL_Delay(2000);
	// vidx = CurrSchedIdx & 0x3;
	// vidx = LastSchedIdx & 0x3;

	stopValve(vidx + 1);
}
#endif

void stopSpecialSequence()
{
	uint8_t vidx = CurrSchedIdx % 12;
	// Stop Pump
	stopPump();
	LCD_Gotoxy(0, 0);
	LCD_Puts("PUMP STOPPED     ");

	// delay 2 secs
	HAL_Delay(2000);

	// Get last working valve or scheduled valve Vn
	// Stop valve
	stopValve(0);
	LCD_Gotoxy(0, 1);
	sprintf(msg, "VALVE %d STOPPED   ", vidx + 1);
	LCD_Puts(msg);
}

// void stopSequence( scheduledPlan)
void stopSequence()
{
	uint8_t vidx = CurrSchedIdx % 12;
	// Stop Pump
	stopPump();
	LCD_Gotoxy(0, 0);
	LCD_Puts("PUMP STOPPED     ");

	// delay 2 secs
	HAL_Delay(2000);

	// Get last working valve or scheduled valve Vn
	// Stop valve
	// stopValve(vidx+1);
	stopValve(0);
	LCD_Gotoxy(0, 1);
	sprintf(msg, "VALVE %d STOPPED   ", vidx + 1);
	LCD_Puts(msg);
}

void stopSequenceTestMode()
{
	uint8_t vidx = CurrSchedIdx % 12;
	// Stop Pump
	stopPump();
	LCD_Clear();
	LCD_Gotoxy(0, 0);
	LCD_Puts("PUMP STOPPED     ");

	// delay 2 secs
	HAL_Delay(2000);

	// Get last working valve or scheduled valve Vn
	// Stop valve
	// stopValve(vidx+1);
	stopValve(0);
	LCD_Gotoxy(0, 1);
	sprintf(msg, "VALVE %d STOPPED   ", vidx + 1);
	LCD_Puts(msg);
	HAL_Delay(2000);
}

void updateSchedule()
{
	uint16_t offsettimeMins = 0;
	int16_t tempvar = 0;
	uint16_t scheduleWholeSum = 0;
	int vidx, i;
	// Read old schedule;
	// Make pointer to next
	// Update Schedule

	// get RTC time
	// HAL_RTC_GetTime();
	CurrSchdlValve_StarttimeMins = todaySchdValveSttimeMins[CurrSchedIdx];
	//mnn[1] = CurrSchdlValve_StarttimeMins; // ##

	// Find first nonzero dur valve
	for (i = 0; i < 12; i++)
	{
		if (masterSchdValveDurationMins[i] > 0)
		{
			FirstSchedIdxPgmA = i;
			// CurrSchedIdx = FirstSchedIdxPgmA;
			break;
		}
	}

	// Find first nonzero dur valve
	for (i = FirstSchedIdxPgmA; i < 12; i++)
	{
		if (masterSchdValveDurationMins[i] != 0)
		{
			LastSchedIdxPgmA = i;
			// CurrSchedIdx = FirstSchedIdx;
			// break;
		}
	}
	for (i = 24; i < 36; i++)
	{
		if (masterSchdValveDurationMins[i] > 0)
		{
			FirstSchedIdxPgmB = i - 24;
			// CurrSchedIdx = FirstSchedIdx;
			break;
		}
	}

	// Find first nonzero dur valve
	for (i = FirstSchedIdxPgmB; i < 36; i++)
	{
		if (masterSchdValveDurationMins[i] != 0)
		{
			LastSchedIdxPgmB = i - 24;
			// CurrSchedIdx = FirstSchedIdx;
			// break;
		}
	}
	// save updated currenttime in EEPROM
	// check last saved time is more than scheduled Valve start time to check whether valve was started or not
	// power came after starttime passed (check for 2 start times from the 2 programs)

	todaySchdValveSttimeMins[CurrSchedIdx] = currRTCtimeMins;

	//mnn[2] = currRTCtimeMins; // ##
	//mnn[3] = LastTimeInstanceMins;
	
	for (vidx = CurrSchedIdx; vidx < valve_count; vidx++)
	{
		scheduleWholeSum += todaySchdValveDurationMins[vidx]; // only for overlapping flag ##
		//mnn[4] += todaySchdValveDurationMins[vidx]; // ##
	}

	// ## Set next day overlapping flag
	if (currRTCtimeMins + scheduleWholeSum >= 1440)
	{
		nextDayOverlapFlag = 1;
	}


	if (pumpstate == (PREWET - 1))
	{
		if (LastTimeInstanceMins - todaySchdPreWetSttimeMins[CurrSchedIdx] >= 0)
		{
			// ## here, this tempvar carry updated dur value
			tempvar = todaySchdPreWetDurMins[CurrSchedIdx] - (LastTimeInstanceMins - todaySchdPreWetSttimeMins[CurrSchedIdx]);
			if (tempvar >= 0)
				todaySchdPreWetDurMins[CurrSchedIdx] = tempvar; //-todaySchdPreWetDurMins[CurrSchedIdx];
			dbgCnt++;
		}
		todaySchdPreWetSttimeMins[CurrSchedIdx] = todaySchdValveSttimeMins[CurrSchedIdx];
		todaySchdFertgSttimeMins[CurrSchedIdx] = todaySchdPreWetSttimeMins[CurrSchedIdx] + todaySchdPreWetDurMins[CurrSchedIdx];
		todaySchdFlushingSttimeMins[CurrSchedIdx] = todaySchdFertgSttimeMins[CurrSchedIdx] + todaySchdFertgDurMins[CurrSchedIdx];
	}

	if (pumpstate == (FERTIGATION - 1))
	{
		if (LastTimeInstanceMins - todaySchdPreWetSttimeMins[CurrSchedIdx] >= 0)
		{
			tempvar = todaySchdFertgDurMins[CurrSchedIdx] - (LastTimeInstanceMins - todaySchdFertgSttimeMins[CurrSchedIdx]);
			if (tempvar >= 0)
				todaySchdFertgDurMins[CurrSchedIdx] = tempvar; //-todaySchdPreWetDurMins[CurrSchedIdx];
		}
		todaySchdFertgSttimeMins[CurrSchedIdx] = todaySchdValveSttimeMins[CurrSchedIdx];
		todaySchdFlushingSttimeMins[CurrSchedIdx] = todaySchdFertgSttimeMins[CurrSchedIdx] + todaySchdFertgDurMins[CurrSchedIdx];
	}

	if (pumpstate == (FLUSHING - 1))
	{
		if (LastTimeInstanceMins - todaySchdPreWetSttimeMins[CurrSchedIdx] >= 0)
		{
			tempvar = todaySchdFlushingDurMins[CurrSchedIdx] - (LastTimeInstanceMins - todaySchdFlushingSttimeMins[CurrSchedIdx]);
			if (tempvar >= 0)
				todaySchdFlushingDurMins[CurrSchedIdx] = tempvar; //-todaySchdPreWetDurMins[CurrSchedIdx];
		}

		todaySchdFlushingSttimeMins[CurrSchedIdx] = todaySchdValveSttimeMins[CurrSchedIdx];  // ## st time
	}

	// ## during mains_off CurrSchedIdx value stored in eprom after mains_on that current CurrSchedIdx duration value is updated, only for irrg ##
	if (LastTimeInstanceMins - CurrSchdlValve_StarttimeMins >= 0)
	{
		tempvar = todaySchdValveDurationMins[CurrSchedIdx] - (LastTimeInstanceMins - CurrSchdlValve_StarttimeMins);
		//mnn[5] = tempvar; // ##
		
		if (tempvar >= 0)
			todaySchdValveDurationMins[CurrSchedIdx] = tempvar;  // ## dur
	}

	// this update the whole todaySchd start time(not dur) for both irrg and fert ##
	for (vidx = CurrSchedIdx + 1; vidx < valve_count; vidx++)  
	{
		// if (todaySchdValveSttimeMins[vidx] !=0)
		{
			// ## this todaySchdValveDurationMins[CurrSchedIdx] = tempvar = todaySchdValveDurationMins[vidx - 1];
			// ## todaySchdValveSttimeMins[CurrSchedIdx] = currRTCtimeMins;
			if ((currRTCtimeMins > todaySchdValveSttimeMins[vidx]) || (todaySchdValveSttimeMins[vidx] < (todaySchdValveSttimeMins[vidx - 1] + todaySchdValveDurationMins[vidx - 1])))
			{
				todaySchdValveSttimeMins[vidx] = todaySchdValveSttimeMins[vidx - 1] + todaySchdValveDurationMins[vidx - 1];

				// if(todaySchdValveSttimeMins[vidx] >=1440)
				//{
				//    todaySchdValveSttimeMins[vidx] = todaySchdValveSttimeMins[vidx]-1440;
				//}

				todaySchdPreWetSttimeMins[vidx] = todaySchdValveSttimeMins[vidx];
				todaySchdFertgSttimeMins[vidx] = todaySchdPreWetSttimeMins[vidx] + todaySchdPreWetDurMins[vidx];
				todaySchdFlushingSttimeMins[vidx] = todaySchdFertgSttimeMins[vidx] + todaySchdFertgDurMins[vidx];
			}
		}
	}
}

int checkflagsPerSec()
{
	if (mains_status == 1 && lcd_on_flag == 0)
	{
		lcd_on();
	}
	int FirstIdx, LastIdx;
	// set startup_Flag
	//  If new starttime reached (4 start time)
	//  If Power restored (SpecialEv_ON_Flag changes from 1 to 0)
	//  If Low_No_Flow_Signal_Flag  changes from 1 to 0
	//  If Rain_Signal_Flag changes from 1 to 0
	//  if (!startup_Flag && !Run_State && (Specialstate!=1))
	if (effectiveFromToday == 1)
	{
		if (!startup_Flag && !Run_State && (Specialstate != 1))
		{
			if (CurrSchedIdx < 24)
			{
				FirstIdx = FirstSchedIdxPgmA;
				LastIdx = LastSchedIdxPgmA;
			}
			else
			{
				FirstIdx = FirstSchedIdxPgmB;
				LastIdx = LastSchedIdxPgmB;
			}


			if (((CurrSchedIdx % 12) == (FirstIdx % 12)) || (SpecialEv_OFF_Flag))
			{
				SpecialEv_OFF_Flag = 0; // what if SpecialEv_Flag was already 0 (No issue)
										// if valve start time is reached
				if ((((weekdays & 0x01) == 1) || (((weekdays >> dayIdx) & 0x01) == 1)) && (ScheduleComplete == 0))
					if ((currRTCtimeMins >= (todaySchdValveSttimeMins[CurrSchedIdx])) && (todaySchdValveDurationMins[CurrSchedIdx] > 0) && (todaySchdValveSttimeMins[CurrSchedIdx] > 0))
					{
						startup_Flag = 1;
						// halt_alloperation =0;
						// scheduleToBeUpdated_Flag=0;
					}


				if (pendIrrgFlag == 1)
				{
					startup_Flag = 1;
				}
			}
			
		}

		//    running_Flag
		//  Pump and currently scheduled Valve started => Set
		//  If Last Valve in each starttime period     => Reset
		//  if any of 3 cases occured                  => Reset

		//    updated_flag
		//    other than all above cases
		if (!startup_Flag && Run_State)
		{
			if (CurrSchedIdx <= (12 + LastSchedIdxPgmA))
			{
				FirstIdx = FirstSchedIdxPgmA;
				LastIdx = LastSchedIdxPgmA;
			}
			else
			{
				FirstIdx = FirstSchedIdxPgmB;
				LastIdx = LastSchedIdxPgmB;
			}
		
			// figure out ##
			if (Specialstate == 1)
			{
				running_Flag = 0;
				updated_flag = 0;
			}
			else  // for Specialstate ==0 ##
			{
				// 0x1F =31
				// if ((CurrSchedIdx & 0x1F) == (LastIdx & 0x1F))
				if ((CurrSchedIdx % 12) == (LastIdx % 12))
				{
					// checking for completion of last(4th) valve
					// if(todaySchdValveDurationMins[CurrSchedIdx]>0)
					{
						if (currRTCtimeMins >= (todaySchdValveSttimeMins[CurrSchedIdx] + todaySchdValveDurationMins[CurrSchedIdx]))
						{
							running_Flag = 0;
							updated_flag = 0;

							// CurrSchedIdx = ((CurrSchedIdx + 4) >> 2) << 2;
							CurrSchedIdx = ((CurrSchedIdx + 12) / 12) * 12; // @@ 0--11=12, 12--23=24, 24--35=36, 36--47=48
							if (CurrSchedIdx == 24)
							{
								CurrSchedIdx += FirstSchedIdxPgmB;
							}
							// else if ((masterSchdPumpSttimeMins[CurrSchedIdx >> 4] != 0))
							else if ((masterSchdPumpSttimeMins[CurrSchedIdx / 12] != 0)) // @@
							{
								CurrSchedIdx += FirstIdx;
							}

							// if PGMA not configured
							// if ((masterSchdPumpSttimeMins[CurrSchedIdx >> 4] == 0) && (CurrSchedIdx < 24))
							if ((masterSchdPumpSttimeMins[CurrSchedIdx / 12] == 0) && (CurrSchedIdx < 24)) // @@
							{
								CurrSchedIdx = 24 + FirstSchedIdxPgmB;
							}
							//		if (CurrSchedIdx>= 8+LastSchedIdxPgmB)
							//		{
							//		   if(masterSchdPumpSttimeMins[0]!=0)
							//			 {
							//			    CurrSchedIdx = FirstSchedIdxPgmA;
							//			 }
							//			 else
							//			 {
							//			    CurrSchedIdx = FirstSchedIdxPgmB;
							//			 }
							//		}
						}
					}
					//		    {
					//						uint8_t  *Data;
					//						Data = &CurrSchedIdx;
					//						EEPROM_WriteNBytes(EP_CURRSCHIDX      , Data, 1); // Error Flag
					//				 }
				}
				// if ((currRTCtimeMins >= todaySchdValveSttimeMins[CurrSchedIdx+1]) && (todaySchdValveSttimeMins[CurrSchedIdx+1]>0))
				// mnn[1] = CurrSchdlValve_StarttimeMins; // ## todaySchdValveSttimeMins[CurrSchedIdx] + tempvar --> see updateSchedule()@@ ##
				else if (currRTCtimeMins >= (todaySchdValveSttimeMins[CurrSchedIdx] + todaySchdValveDurationMins[CurrSchedIdx]))
				// if schedule for next valve has come (we are assuming the schedule timing is validated and non-overlapping)
				{
					NextSchedIdx = CurrSchedIdx + 1; // updated the sched ##
					if (NextSchedIdx >= valve_count)
					{
						NextSchedIdx = 0; // when NextSchedIdx = CurrSchedIdx + 1; reach to last valve of valve_count ##
					}
					else
					{
						updated_flag = 1;
						// if (todaySchdValveDurationMins[CurrSchedIdx] >0)
						//   LastSchedIdx = CurrSchedIdx;
					}
					{
						uint8_t *Data;
						Data = &NextSchedIdx;
						EEPROM_WriteNBytes(EP_CURRSCHIDX, Data, 1); // ## due to updated_flag = 1; --> callback fn do --> CurrSchedIdx = NextSchedIdx; and updated_flag =0; 
					}
				}
			}
		}
	}

	
	
	// if any of 3 flags get set
	if (startup_Flag == 1)
	{
		if (!scheduleToBeUpdated_Flag)
		{
			scheduleToBeUpdated_Flag = 1;
			// save timing in EEPROM
			// LastTimeInstanceMins = 0;
			// printf("   Power came after starttime passed \n");
		}
	}
	
	
	if (Specialstate == ON)  // here specialstate means contains all error check failures  ##
	{
		if (SpecialEv_ON_Flag == 1)
		{
			scheduleToBeUpdated_Flag = 1;
			SpecialEv_ON_Flag = 0;
			// save timing in EEPROM
			// LastTimeInstanceMins = currRTCtimeMins;
			if (/*pumpstate !=1 ||*/ pumpstate != 0)
			{
				pumpstate--;
			}
		}

		if ((mains_status == 0) && (mains_off_Signal_Flag == 1))
		{
			// LCD_Clear();
			// LCD_Gotoxy(0,1);
			// LCD_Puts("MAINS OFF   ");
			// uct_EEPROM_UPDATEDWRITE();
			// HAL_Delay(1000);
			errorFlag |= 1 << MAINS_OFF;
		}
		HAL_Delay(1000);
		if (mains_status == 1) // && (mains_off_Signal_Flag ==1)
		{
			uint8_t tempFlag = 0xFD; //!(1<<MAINS_OFF);
									 // LCD_Clear();
									 // LCD_Gotoxy(0,1);
									 // LCD_Puts("MAINS ON");
			errorFlag &= tempFlag;
		}

		if ((rain_status == 1) && (Rain_Signal_Flag == 1))
		{
			//			LCD_Clear();
			//			LCD_Gotoxy(0,1);
			//			LCD_Puts("RAIN SHUT OFF   ");
			//			HAL_Delay(1000);
			errorFlag |= 1 << RAIN_SHUT_OFF;
			// HAL_Delay(1000);
		}
		if ((rain_status == 0) && (Rain_Signal_Flag == 1))
		{
			uint8_t tempFlag = 0XFE; //!(1<<RAIN_SHUT_OFF);
									 //			LCD_Clear();
									 //			LCD_Gotoxy(0,1);
									 //			LCD_Puts("RAIN STOPPED");
			errorFlag &= tempFlag;
		}

		//		if((disp == 0) && (disp_cnfg==1))
		//		{
		//	//		uint8_t tempFlag = 0XFE;//!(1<<RAIN_SHUT_OFF);
		//			LCD_Clear();
		//			LCD_Gotoxy(0,1);
		//			LCD_Puts("DISPLAY MODE END");
		//			disp_cnfg = 0;
		////			Rain_Signal_Flag=0;
		//		}

		// for low flow check two times ##
		if ((ScheduleComplete == 0) && (Low_No_Flow_Signal_Flag) && (rain_status == 0) && (mains_status == 1) && (halt_alloperation != 1))
		{
			LCD_Clear();
			LCD_Gotoxy(0, 1);
			if (low_no_flow_count <= 1)
			{
				sprintf(msg, "RECHARGE STATE %d", low_no_flow_count + 1);
				LCD_Puts(msg);
				//			  HAL_Delay(1000);
				HAL_Delay(1000); // changed by rishabh
			}
			if (low_no_flow_count >= 2)
			{
				uint8_t Data;
				uint8_t *pData;
				uint8_t tempFlag = 0XDF; //!(1<<PUMP_RECHARGING_TIME_ACTIVE);
				errorFlag &= tempFlag;
				halt_alloperation = 1;
				SecondRTTimeInstanceMins = currRTCtimeMins;
				SecondRTdayIdx = dayIdx;
				errorFlag |= 1 << LOW_FLOW_2ND_RT;
				Data = 1 << LOW_FLOW_2ND_RT;
				EEPROM_WriteNBytes(EP_ERRORFLAG, &Data, 1); // Error Flag

				pData = (uint8_t *)&SecondRTTimeInstanceMins;
				for (uint8_t idx = 0; idx < 2; idx++)
				{
					EEPROM_WriteByte(EP_2ND_RT_INSTMINS + idx, pData[idx]);
				}

				pData = (uint8_t *)&SecondRTdayIdx;
				EEPROM_WriteByte(EP_2ND_RT_DAY, pData[0]);

				LCD_Clear();
				//					  LCD_Gotoxy(0,1);
				//					  LCD_Puts("LOW FLOW 2ND RT ");
				//					  HAL_Delay(1000);
				low_no_flow_count = 0;
				Specialstate = OFF;
				SpecialEv_OFF_Flag = 1;
				// low_flow_status1 = 1;
				Low_No_Flow_Signal_Flag = 0;
				gvCommand = 0X00;
				valve_output(&gvCommand,0x00); // ## fert--@@

				// CurrSchedIdx = 	(CurrSchedIdx/4 + 1)*4;
				// NextSchedIdx = CurrSchedIdx;
				// break;
				//  HALT ALL\\AUTOMAT_LCA_PINMAPING\../Core/Src/config.c\pumpRechargingTimeAbsMins Operations
			}
			if (low_no_flow_count > 0)
			{
				errorFlag |= 1 << PUMP_RECHARGING_TIME_ACTIVE;
			}

			if ((pumpRechargingTimeCnt >= pumpRechargingTimeAbsMins)) // && (pumpRechargingTimeAbsMins !=0))
			{
				Specialstate = OFF;
				SpecialEv_OFF_Flag = 1;
				pump_initiation = 1;
				Low_No_Flow_Signal_Flag = 0;
				low_no_flow_count++;
				// low_flow_status =0;
				// low_flow_status1 = 1;
				// LCD_Puts("INIT STATE ");
				// HAL_Delay(200);
			}

			// scheduleToBeUpdated_Flag =1;
		}
	}


	// ##
	if (ScheduleComplete==1)
	{
		balMins_msg=0;
	}

}

int callback_scheduledtimerInterrupt_5mins()
{
	uint16_t timeMins = 0;
	uint8_t i;

	// Actions as per updated_Flag and startup_Flag
	if (startup_Flag && !Run_State) // it need to be 1 && !0, here, Run_state is 0 , means no valve is running  ##
	{
		if (scheduleToBeUpdated_Flag)  // it need to be 1  ##
		{
			updateSchedule();
			scheduleToBeUpdated_Flag = 0;
		}
		
		// check frtigation mode supported by this valve
		if (irrigationMode[CurrSchedIdx / 24] == FERTIGATION_MODE)
		{
			if (currRTCtimeMins >= todaySchdValveSttimeMins[CurrSchedIdx])
			{
				statePrewetSetDone = 0;
				stateFertigationSetDone = 0;
				stateFlushingSetDone = 0;
				pumpstate = 0;
			}
		}
		
		// start valve, so, Run_state is 1 ##
		if (todaySchdValveDurationMins[CurrSchedIdx] != 0)
			startSequence();
		

		running_Flag = 1;
		startup_Flag = 0;  // ## when Run_State=1, means pg runing then startup_Flag=0  // to be set at each start time and to reset just after it
		Run_State = 1;
	}

	// if mains power is lost, rain , low_flow --> stop the valve ##
	if (Specialstate == 1)
	{
		stopSequence();
		Run_State = 0;
	}

	// for stopping the sequence
	// if Specialstate == 1 then running_Flag == 0 , so here, to run this if block, Run_state =1 and then set to 0 ##
	if ((running_Flag == 0) && Run_State)
	{
		stopSequence();

		if ((currRTCtimeMins >= (todaySchdFlushingSttimeMins[CurrSchedIdx] + todaySchdFlushingDurMins[CurrSchedIdx])) && !stateFlushingSetDone)
		{
			stateFlushingSetDone = 1;
			pumpstate = 0;
		}
		Run_State = 0;  // set Run_State --> 0  ##
		
		// if Specialstate =0 ,  means mains available, no low flow and rain error  ## 
		if (!Specialstate)
		{
		
			if (CurrSchedIdx >= valve_count)
			{
				uint8_t tempFlag = 0XEF; //!(1<<PENDG_IRRG);
				errorFlag &= tempFlag;
				// Find first nonzero dur valve
				if (masterSchdPumpSttimeMins[0] != 0)
					CurrSchedIdx = FirstSchedIdxPgmA;
				else
					CurrSchedIdx = 24 + FirstSchedIdxPgmB;
					// CurrSchedIdx = 0;
					// ScheduleComplete=1;

				// for pending stuff, ##
				if (nextDayOverlapFlag == 1)
				{
					if (currRTCtimeMins >= 1440)  // 24*60 =1440
						currRTCtimeMins = currRTCtimeMins - 1440;  // if this block run then, this value is 0 , ##

					dayIdx++;
					if (dayIdx > 7)
					{
						dayIdx = 1;
					}

					// check next
					// this means (weekdays & 0x01) == 1) --> all days, means MON to SUN, dayIdx value varies 1 to 8 , select one day  ##
					// if weekdays = 0 then (that day not programmed)  ##
					if (((weekdays & 0x01) == 1) || (((weekdays >> dayIdx) & 0x01) == 1))
					{
						createTodaySchedule(&IrrigationPgm[0]);  // create for pgA  ##
						ScheduleComplete = 0;
					}

					// if weekdaysPgmA = 1 then all-days, so this block for single days
					if ((weekdaysPgmA & 0x01) != 1)
					{
						for (i = dayIdx; i < 8; i++)
						{
							if (((weekdaysPgmA >> i) & 0x01) == 1)
							{
								NextSchedDayPgmA = i;
								break;
							}
						}
					}
					else
					{
						NextSchedDay = dayIdx;
					}

					// if weekdaysPgmB = 1 then all-days, so this block for single days
					if ((weekdaysPgmB & 0x01) != 1)
					{
						for (i = dayIdx; i < 8; i++)
						{
							if (((weekdaysPgmB >> i) & 0x01) == 1)
							{
								NextSchedDayPgmB = i;
								break;
							}
						}
					}
					else
					{
						NextSchedDay = (dayIdx % 7) + 1;
					}

					if (((weekdaysPgmA & 0x01) == 1) || ((weekdaysPgmB & 0x01) == 1))
					{
						NextSchedDay = dayIdx;
					}
					else
					{

						NextSchedDay = NextSchedDayPgmA;

						if ((NextSchedDayPgmA >= NextSchedDayPgmB) && (NextSchedDayPgmB > 0))
						{
							NextSchedDay = NextSchedDayPgmB;
						}
						else if ((NextSchedDayPgmB > NextSchedDayPgmA) && (NextSchedDayPgmA > 0))
						{
							NextSchedDay = NextSchedDayPgmA;
						}
					}

					// reset next day overlap flag
					// nextDayOverlapFlag = 0;
					// pendIrrgFlag =0;

					// ScheduleComplete=1;

					// change by sunny
					// if(ScheduleComplete == 1)
					{
						uint8_t *Data;
						pendIrrgFlag = 0;
						Data = (uint8_t *)&pendIrrgFlag;
						EEPROM_WriteNBytes(EP_PENDIRRG, Data, 1); // pendIrrgFlag Flag

						Data = &CurrSchedIdx;
						EEPROM_WriteNBytes(EP_CURRSCHIDX, Data, 1); // when pending flag (nextDayOverlapFlag==1) occur and ScheduleComplete is not complete then CurrSchedIdx value stored in eprom ##
					}

					nextDayOverlapFlag = 0;
					ScheduleComplete = 0;
					// pendIrrgFlag = 0;
				}
				else
				{
					uint8_t *Data;

					ScheduleComplete = 1;
					Data = &ScheduleComplete;
					EEPROM_WriteNBytes(EP_SCHEDULE_COMPLETE, Data, 1); // Error Flag
					createNextSchedule(&IrrigationPgm[0]);

					// this inner scope is required , otherwise error occur ##
					{ 
						uint8_t *Data;
						Data = &CurrSchedIdx;
						EEPROM_WriteNBytes(EP_CURRSCHIDX, Data, 1); // when ScheduleComplete is complete occur then CurrSchedIdx value stored in eprom ##
					}
				}
			}

			// ## if there is pendIrrgFlag=1, then (prevWeekDay != dayIdx) is true and this block runs
			else if (prevWeekDay != dayIdx)
			{
				sprintf(msg, "PENDG IRRG **%3s", week[prevWeekDay - 1]);
				errorFlag |= 1 << PENDG_IRRG;
				// prevWeekDay = dayIdx;
				LCD_Clear();
				LCD_Gotoxy(0, 0);
				LCD_Puts(msg);
			}
			//				if(prevWeekDay != dayIdx)
			//				{
			//					 prevWeekDay = dayIdx;
			//				}
		}
	}

	// Actions as per updated_flag
	// ## while pg runing and if update is required 
	if (updated_flag && Run_State)
	{
		if (todaySchdValveDurationMins[CurrSchedIdx + 1] != 0)
			updateValveSequence();

		CurrSchedIdx = NextSchedIdx;
		//			 if (CurrSchedIdx <= valve_count)
		//			 {
		//					uint8_t  *Data;
		//					Data = &CurrSchedIdx;
		//					EEPROM_WriteNBytes(EP_CURRSCHIDX      , Data, 1); // Error Flag
		//			 }

		// check fertigation mode supported by this valve
		if (irrigationMode[CurrSchedIdx / 24] == FERTIGATION_MODE)
		{
			if (currRTCtimeMins >= todaySchdValveSttimeMins[CurrSchedIdx])
			{
				statePrewetSetDone = 0;
				stateFertigationSetDone = 0;
				stateFlushingSetDone = 0;
				pumpstate = 0;
			}
		}

		updated_flag = 0;

		// saveSeqContext();// save running valve idx, starttime, next schedule idx, start time
	}
#if 0
	while (Low_No_Flow_Signal_Flag)
	{  
		//wait for recharging time
		low_no_flow_count++;

		if(low_no_flow_count >=2)
		{
			errorFlag[1] = 1;
			//printf("Low-No Flow 2nd RT\n\n");
			scheduleToBeUpdated_Flag =1;
			break;
		} 
		scheduleToBeUpdated_Flag =1;
	}
#endif
	// take care of fertigation mode
	// Fertigation Valve	 Bypass
	// Prewet	                0	          1
	// Fertigation	            1	          0
	// Flush	                    0	          1

	// for only fert ##
	if (Run_State)
	{
		// check frtigation mode supported by this valve
		if (irrigationMode[CurrSchedIdx / 24] == FERTIGATION_MODE)
		{
			if (todaySchdValveDurationMins[CurrSchedIdx] != 0)
			{

				if ((currRTCtimeMins >= todaySchdPreWetSttimeMins[CurrSchedIdx]) && (pumpstate == (PREWET - 1)) && !statePrewetSetDone)
				{
					pumpstate = PREWET;
					startPrewetByPass(CurrSchedIdx);
				}
				if ((currRTCtimeMins >= (todaySchdPreWetSttimeMins[CurrSchedIdx] + todaySchdPreWetDurMins[CurrSchedIdx])) && (!statePrewetSetDone))
				{
					statePrewetSetDone = 1;
				}

				if ((currRTCtimeMins >= todaySchdFertgSttimeMins[CurrSchedIdx]) && (pumpstate == PREWET) && (!stateFertigationSetDone))
				{
					pumpstate = FERTIGATION;
					startFertigation(CurrSchedIdx);
				}
				if ((currRTCtimeMins >= (todaySchdFertgSttimeMins[CurrSchedIdx] + todaySchdFertgDurMins[CurrSchedIdx])) && (!stateFertigationSetDone))
				{
					stateFertigationSetDone = 1;
				}

				if ((currRTCtimeMins >= todaySchdFlushingSttimeMins[CurrSchedIdx]) && (pumpstate == FERTIGATION) && !stateFlushingSetDone)
				{
					pumpstate = FLUSHING;
					startFlushByPass(CurrSchedIdx);
				}

				if ((currRTCtimeMins >= (todaySchdFlushingSttimeMins[CurrSchedIdx] + todaySchdFlushingDurMins[CurrSchedIdx])) && !stateFlushingSetDone)
				{
					stateFlushingSetDone = 1;
					pumpstate = 0;
				}
			}
		}
	}
}

void runtestSeq()
{
	// Valve 1,  PUMP and BP
	//	 running = 0xD0;
	//	   valve_output(&running);
	LCD_Gotoxy(0, 0);
	LCD_Puts("V1 PUMP ON     ");
	LCD_Gotoxy(0, 1);
	LCD_Puts("PREWET ON      ");
	HAL_Delay(60000);

	// Valve 1,  PUMP and F
	running = 0xB0;
	valve_output(&running,0x00); // fert-@@
	LCD_Gotoxy(0, 0);
	LCD_Puts("V1 PUMP ON     ");
	LCD_Gotoxy(0, 1);
	LCD_Puts("FERTIGATION ON ");
	HAL_Delay(60000);

	// Valve 1,2,  PUMP and Bypass
	running = 0xD8;
	valve_output(&running,0x00); // fert-@@
	LCD_Gotoxy(0, 0);
	LCD_Puts("V2 PUMP ON     ");
	LCD_Gotoxy(0, 1);
	LCD_Puts("PREWET ON      ");
	//		 LCD_Gotoxy(0,0);
	//		 LCD_Puts("V1 PUMP STOPPED ");
	//		 LCD_Gotoxy(0,1);
	//		 LCD_Puts("FERTIGATION ON ");
	HAL_Delay(3000);

	// Valve 2 and PUMP, BP
	running = 0xC8;
	valve_output(&running,0x00); // fert-@@
	HAL_Delay(60000);

	// Valve 2 and PUMP, F
	running = 0xA8;
	valve_output(&running,0x00); // fert-@@
	LCD_Gotoxy(0, 0);
	LCD_Puts("V2 PUMP ON     ");
	LCD_Gotoxy(0, 1);
	LCD_Puts("FERTIGATION ON ");
	HAL_Delay(60000);

	// Valve 2,3 and PUMP, Bypass
	running = 0xCC;
	valve_output(&running, 0x00); // fert-@@
	LCD_Gotoxy(0, 0);
	LCD_Puts("V3 PUMP ON     ");
	LCD_Gotoxy(0, 1);
	LCD_Puts("PREWET ON      ");
	//		 LCD_Gotoxy(0,0);
	//		 LCD_Puts("V2 PUMP STOPPED ");
	//		 LCD_Gotoxy(0,1);
	//		 LCD_Puts("FERTIGATION ON ");
	HAL_Delay(3000);

	// Valve 3 and PUMP, BP
	running = 0xC4;
	valve_output(&running,0x00); // fert-@@
	HAL_Delay(60000);

	// Valve 3 and PUMP, F
	running = 0xA4;
	valve_output(&running,0x00); // fert-@@
	LCD_Gotoxy(0, 0);
	LCD_Puts("V3 PUMP ON     ");
	LCD_Gotoxy(0, 1);
	LCD_Puts("FERTIGATION ON ");
	HAL_Delay(60000);

	// Valve 3,4 and PUMP, Bypass
	running = 0xC6;
	valve_output(&running,0x00); // fert-@@
	LCD_Gotoxy(0, 0);
	LCD_Puts("V4 PUMP ON     ");
	LCD_Gotoxy(0, 1);
	LCD_Puts("PREWET ON      ");
	//		 LCD_Gotoxy(0,0);
	//		 LCD_Puts("V3 PUMP STOPPED ");
	//		 LCD_Gotoxy(0,1);
	//		 LCD_Puts("FERTIGATION ON ");
	HAL_Delay(3000);

	// Valve 4 and PUMP, BP
	running = 0xC2;
	valve_output(&running,0x00); // fert-@@
	HAL_Delay(60000);

	// Valve 4 and PUMP, F
	running = 0xA2;
	valve_output(&running,0x00); // fert-@@
	LCD_Gotoxy(0, 0);
	LCD_Puts("V4 PUMP ON     ");
	LCD_Gotoxy(0, 1);
	LCD_Puts("FERTIGATION ON ");
	HAL_Delay(60000);

	CurrSchedIdx = 3;
	stopSequenceTestMode();
}

#if 0
void test_mode()
{
	test_mode_flag = 1;
	int mains_status, rain_status1, low_flow_status1;

	mains_status = HAL_GPIO_ReadPin(MAINS_SN_GPIO_Port, MAINS_SN_Pin);
	rain_status1 = HAL_GPIO_ReadPin(SENSOR_1_GPIO_Port, SENSOR_1_Pin);
	low_flow_status1 = HAL_GPIO_ReadPin(SENSOR_2_GPIO_Port, SENSOR_2_Pin);

	if (mains_status == 1)
	{
		LCD_Gotoxy(0, 0);
		LCD_Puts("MAINS ON       ");
		HAL_Delay(2000);

		if (rain_status1 == 0)
		{
			LCD_Gotoxy(0, 0);
			LCD_Puts("RAIN ON        ");
			HAL_Delay(2000);
		}
		else
		{
			LCD_Gotoxy(0, 0);
			LCD_Puts("RAIN OFF       ");
			HAL_Delay(5000);
		}

		startSequenceTestMode();

		if (low_flow_status1 == 1)
		{
			LCD_Gotoxy(0, 0);
			LCD_Puts("LOW FLOW ON    ");
			HAL_Delay(2000);
		}
		else
		{
			LCD_Gotoxy(0, 0);
			LCD_Puts("LOW FLOW OFF   ");
			HAL_Delay(5000);

			runtestSeq();
		}
	}
	else
	{
		LCD_Gotoxy(0, 0);
		LCD_Puts("MAINS OFF      ");
		HAL_Delay(5000);
	}
	test_mode_flag = 0;
}



void test_mode()
{
	gvCommand=0x00;
	gvCommand_pcf=0x02;
	
	valve_output_test(&gvCommand, &gvCommand_pcf);
	
	LCD_Gotoxy(0, 0);
	LCD_Puts("PCF TEST    ");
	LCD_Gotoxy(0, 1);
	LCD_Puts("V5 ON     ");
	HAL_Delay(20000);
	
	gvCommand_pcf=0x04;
	valve_output_test(&gvCommand, &gvCommand_pcf);
	LCD_Gotoxy(0, 0);
	LCD_Puts("PCF TEST    ");
	LCD_Gotoxy(0, 1);
	LCD_Puts("V6 ON     ");
	HAL_Delay(20000);
	
	gvCommand_pcf=0x08;
	valve_output_test(&gvCommand, &gvCommand_pcf);
	LCD_Gotoxy(0, 0);
	LCD_Puts("PCF TEST    ");
	LCD_Gotoxy(0, 1);
	LCD_Puts("V7 ON     ");
	HAL_Delay(20000);
	
	gvCommand_pcf=0x01;
	valve_output_test(&gvCommand, &gvCommand_pcf);
	LCD_Gotoxy(0, 0);
	LCD_Puts("PCF TEST    ");
	LCD_Gotoxy(0, 1);
	LCD_Puts("V8 ON     ");
	HAL_Delay(20000);
	
	gvCommand_pcf=0x10;
	valve_output_test(&gvCommand, &gvCommand_pcf);
	LCD_Gotoxy(0, 0);
	LCD_Puts("PCF TEST    ");
	LCD_Gotoxy(0, 1);
	LCD_Puts("V9 ON     ");
	HAL_Delay(20000);
	
	gvCommand_pcf=0x80;
	valve_output_test(&gvCommand, &gvCommand_pcf);
	LCD_Gotoxy(0, 0);
	LCD_Puts("PCF TEST    ");
	LCD_Gotoxy(0, 1);
	LCD_Puts("V10 ON     ");
	HAL_Delay(20000);
	
	gvCommand_pcf=0x40;
	valve_output_test(&gvCommand, &gvCommand_pcf);
	LCD_Gotoxy(0, 0);
	LCD_Puts("PCF TEST    ");
	LCD_Gotoxy(0, 1);
	LCD_Puts("V11 ON     ");
	HAL_Delay(20000);
	
	gvCommand_pcf=0x20;
	valve_output_test(&gvCommand, &gvCommand_pcf);
	LCD_Gotoxy(0, 0);
	LCD_Puts("PCF TEST    ");
	LCD_Gotoxy(0, 1);
	LCD_Puts("V12 ON     ");
	HAL_Delay(20000);
	
	gvCommand_pcf=0x00;
	valve_output_test(&gvCommand, &gvCommand_pcf);
	HAL_Delay(1000);	
}
#endif

void test_mode()
{
	LCD_Clear();
	sw1=0;
	uint8_t rep_i[4] = {1, 2, 30, 50}; // ##
	char rep_c[4][3] = {"1", "2", "30", "50"}; // ##
	while (1)
	{
	
		if (!lcd_retain_flag1)
		{
			LCD_Clear();
			lcd_retain_flag1 = 1;
		}

		rep_v = rep_i[sw2];
		sw2_max = 4;
		LCD_Gotoxy(0, 0);
		LCD_Puts("Select Times: ");
		LCD_Gotoxy(0, 1);
		sprintf(msg, "%s     ", rep_c[sw2]);
		LCD_Puts(msg);		


		if ((sw1 == 1) && (input_timeout < 1000))
		{
			MenuDoneCnt = 4;
			sw1=0;
		}
		
		if (MenuDoneCnt == 4)
			while (1)
			{
				pcf_return_inbuilt=pcf_inbuilt_on(0x00);
				LCD_Gotoxy(0, 0);
				LCD_Puts("PUMP START   ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("ON     ");
				HAL_GPIO_WritePin(RELAY_PWR_GPIO_Port,RELAY_PWR_Pin,0);
				HAL_Delay(200);
				HAL_GPIO_WritePin(PUMP_R_GPIO_Port,PUMP_R_Pin,1);
				HAL_Delay(1000);
				
				pcf_return_inbuilt = pcf_inbuilt_on(0x68);
				LCD_Gotoxy(0, 0);
				LCD_Puts("PCF TEST    ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("V1 ON     ");
				HAL_Delay(30000);
				
				pcf_return_inbuilt = pcf_inbuilt_on(0x64);
				LCD_Gotoxy(0, 0);
				LCD_Puts("PCF TEST    ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("V2 ON     ");
				HAL_Delay(30000);
				
				pcf_return_inbuilt = pcf_inbuilt_on(0x62);
				LCD_Gotoxy(0, 0);
				LCD_Puts("PCF TEST    ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("V3 ON     ");
				HAL_Delay(30000);
				
				pcf_return_inbuilt = pcf_inbuilt_on(0x61);
				LCD_Gotoxy(0, 0);
				LCD_Puts("PCF TEST    ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("V4 ON     ");
				HAL_Delay(30000);
				
				pcf_return_inbuilt = pcf_inbuilt_on(0x00);
				HAL_Delay(1000);
				
				pcf_return = pcf_on(0x02);
				LCD_Gotoxy(0, 0);
				LCD_Puts("PCF TEST    ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("V5 ON     ");
				HAL_Delay(30000);
				
				pcf_return = pcf_on(0x04);
				LCD_Gotoxy(0, 0);
				LCD_Puts("PCF TEST    ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("V6 ON     ");
				HAL_Delay(30000);
				
				pcf_return = pcf_on(0x08);
				LCD_Gotoxy(0, 0);
				LCD_Puts("PCF TEST    ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("V7 ON     ");
				HAL_Delay(30000);
				
				pcf_return = pcf_on(0x01);
				LCD_Gotoxy(0, 0);
				LCD_Puts("PCF TEST    ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("V8 ON     ");
				HAL_Delay(30000);
				
				pcf_return = pcf_on(0x10);
				LCD_Gotoxy(0, 0);
				LCD_Puts("PCF TEST    ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("V9 ON     ");
				HAL_Delay(30000);
				
				pcf_return = pcf_on(0x80);
				LCD_Gotoxy(0, 0);
				LCD_Puts("PCF TEST    ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("V10 ON     ");
				HAL_Delay(30000);
				
				pcf_return = pcf_on(0x40);
				LCD_Gotoxy(0, 0);
				LCD_Puts("PCF TEST    ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("V11 ON     ");
				HAL_Delay(30000);
				
				pcf_return = pcf_on(0x20);
				LCD_Gotoxy(0, 0);
				LCD_Puts("PCF TEST    ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("V12 ON     ");
				HAL_Delay(30000);
				
				pcf_return = pcf_on(0x00);
				LCD_Gotoxy(0, 0);
				LCD_Puts("PCF TEST    ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("exp off     ");
				HAL_Delay(2000);
				
				rep_stop++;
	
				if (rep_v == rep_stop)
				{
					MenuDoneCnt = 0;
					sw1=0;
					rep_stop=0;
					HAL_GPIO_WritePin(RELAY_PWR_GPIO_Port,RELAY_PWR_Pin,1);
					HAL_GPIO_WritePin(PUMP_R_GPIO_Port,PUMP_R_Pin,0);
					pcf_return_inbuilt = pcf_inbuilt_on(0x00);
					break;
				}

			}
	}
}



void set_date_day_time()
{
	//	int h = 0 ;
	// sw1=2;
	sw1 = SW_START;

	while ((d_t == 1) && (index1 == 2))
	{
		if ((sw1 == SW_START && input_timeout < 1000))
		{
			if (!lcd_retain_flag1)
			{
				LCD_Clear();
				lcd_retain_flag1 = 1;
			}
			get_year = year[sw2];
			sw2_max = 7;
			LCD_Gotoxy(0, 0);
			LCD_Puts("SET YEAR:         ");
			LCD_Gotoxy(0, 1);
			sprintf(msg, "%d                     ", year[sw2]);
			LCD_Puts(msg);
			// HAL_Delay(1000);
		}

		if (sw1 < 1)
		{
			index1--;
			backflag = 0;
		}

		if ((sw1 == SW_START + 1) && input_timeout < 1000)
		{
			if (!lcd_retain_flag1)
			{
				LCD_Clear();
				lcd_retain_flag1 = 1;
			}
			get_month = month[sw2];
			sw2_max = 12;
			LCD_Gotoxy(0, 0);
			LCD_Puts("SET MONTH:         ");
			LCD_Gotoxy(0, 1);
			sprintf(msg, "%s                    ", months[sw2]);
			LCD_Puts(msg);
			// HAL_Delay(1000);
		}

		if ((sw1 == SW_START + 2 && input_timeout < 1000))
		{
			if (get_month == 1 || get_month == 3 || get_month == 5 || get_month == 7 || get_month == 8 || get_month == 10 || get_month == 12)
			{

				if (!lcd_retain_flag1)
				{
					LCD_Clear();
					lcd_retain_flag1 = 1;
				}
				get_date = date[sw2];
				sw2_max = 31;
				LCD_Gotoxy(0, 0);
				LCD_Puts("SET DATE         ");
				LCD_Gotoxy(0, 1);
				sprintf(msg, "%d                    ", date[sw2]);
				LCD_Puts(msg);
				// HAL_Delay(1000);
			}
			if (get_month == 4 || get_month == 6 || get_month == 9 || get_month == 11)
			{
				if (!lcd_retain_flag1)
				{
					LCD_Clear();
					lcd_retain_flag1 = 1;
				}
				get_date = date[sw2];
				sw2_max = 30;
				LCD_Gotoxy(0, 0);
				LCD_Puts("SET DATE         ");
				LCD_Gotoxy(0, 1);
				sprintf(msg, "%d                    ", date[sw2]);
				LCD_Puts(msg);
				// HAL_Delay(1000);
			}
			if (get_month == 2)
			{
				if (!lcd_retain_flag1)
				{
					LCD_Clear();
					lcd_retain_flag1 = 1;
				}
				get_date = date[sw2];
				sw2_max = 29;
				LCD_Gotoxy(0, 0);
				LCD_Puts("SET DATE         ");
				LCD_Gotoxy(0, 1);
				sprintf(msg, "%d                    ", date[sw2]);
				LCD_Puts(msg);
				// HAL_Delay(1000);
			}
		}

		if ((sw1 == SW_START + 3 && input_timeout < 1000))
		{
			if (!lcd_retain_flag1)
			{
				LCD_Clear();
				lcd_retain_flag1 = 1;
			}
			get_hrs = set_DurHrs[sw2];
			sw2_max = 24;
			LCD_Gotoxy(0, 0);
			LCD_Puts("SET TIME:         ");
			LCD_Gotoxy(0, 1);
			sprintf(msg, "%d HRS                    ", set_DurHrs[sw2]);
			LCD_Puts(msg);
			// HAL_Delay(1000);
		}

		if ((sw1 == SW_START + 4 && input_timeout < 1000))
		{
			if (!lcd_retain_flag1)
			{
				LCD_Clear();
				lcd_retain_flag1 = 1;
			}
			get_mins = mins[sw2];
			sw2_max = 60;
			LCD_Gotoxy(0, 0);
			LCD_Puts("SET TIME:         ");
			LCD_Gotoxy(0, 1);
			sprintf(msg, "%d MINS                    ", mins[sw2]);
			LCD_Puts(msg);
			// HAL_Delay(1000);
		}

		if ((sw1 == SW_START + 5 && input_timeout < 1000))
		{
			if (!lcd_retain_flag1)
			{
				LCD_Clear();
				lcd_retain_flag1 = 1;
			}
			get_day = day[sw2];
			sw2_max = 7;
			LCD_Gotoxy(0, 0);
			LCD_Puts("SET DAY:         ");
			LCD_Gotoxy(0, 1);
			sprintf(msg, "%s                    ", week[sw2]);
			LCD_Puts(msg);
			// HAL_Delay(1000);
		}

		if ((sw1 == SW_START + 6 && input_timeout < 1000))
		{
			LCD_Gotoxy(0, 0);
			LCD_Puts("SAVING  SETTINGS        ");
			LCD_Gotoxy(0, 1);
			LCD_Puts("SAVED..  RESTART              ");
			d_t = 0;
			MX_RTC_Init();
			RTC_TimeShow();
			HAL_Delay(2000);
			HAL_NVIC_SystemReset();
		}
	}
}

void RTC_TimeShow()
{
	LCD_Clear();
	RTC_TimeTypeDef getTime = {0};
	RTC_DateTypeDef getDate = {0};
	HAL_RTC_GetTime(&hrtc, &getTime, RTC_FORMAT_BIN);
	currRTCtimeMins = getTime.Hours * 60 + getTime.Minutes;
	HAL_RTC_GetDate(&hrtc, &getDate, RTC_FORMAT_BIN);
	dayIdx = getDate.WeekDay;
	prevWeekDay = dayIdx;
	LCD_Clear();
	LCD_Gotoxy(0, 0);
	sprintf(lcd_data, "DATE: %02d/%02d/%4d", getDate.Date, getDate.Month, 2000 + getDate.Year);
	LCD_Puts(lcd_data);
	sprintf(lcd_data, "TIME: %02d:%02d  %3s", getTime.Hours, getTime.Minutes, week[getDate.WeekDay - 1]);
	// sprintf(lcd_data,"TIME: %02d:%02d:%02d", getTime.Hours, getTime.Minutes, getTime.Seconds);
	LCD_Gotoxy(0, 1);
	LCD_Puts(lcd_data);
	HAL_Delay(5000);
	memset(lcd_data, 0x00, sizeof(lcd_data));
}

void RTC_TimeShow1()
{
	int hrs, mins;
	LCD_Clear();
	RTC_TimeTypeDef getTime = {0};
	RTC_DateTypeDef getDate = {0};
	hrs = currRTCtimeMins / 60;
	mins = currRTCtimeMins - (hrs * 60);
	//	HAL_RTC_GetTime(&hrtc, &getTime, RTC_FORMAT_BIN);
	//	currRTCtimeMins = getTime.Hours*60 + getTime.Minutes;
	HAL_RTC_GetDate(&hrtc, &getDate, RTC_FORMAT_BIN);
	//  dayIdx = 	getDate.WeekDay;
	//  prevWeekDay = dayIdx;
	LCD_Clear();
	LCD_Gotoxy(0, 0);
	sprintf(lcd_data, "DATE: %02d/%02d/%4d", getDate.Date, getDate.Month, 2000 + getDate.Year);
	LCD_Puts(lcd_data);
	sprintf(lcd_data, "TIME: %02d:%02d  %3s", hrs % 24, mins, week[getDate.WeekDay - 1]);
	// sprintf(lcd_data,"TIME: %02d:%02d:%02d", getTime.Hours, getTime.Minutes, getTime.Seconds);
	LCD_Gotoxy(0, 1);
	LCD_Puts(lcd_data);
	HAL_Delay(1000);
	LCD_Gotoxy(8, 1);
	LCD_Puts(" ");
	HAL_Delay(200);
	LCD_Gotoxy(0, 1);
	sprintf(lcd_data, "TIME: %02d:%02d  %3s", hrs % 24, mins, week[getDate.WeekDay - 1]);
	LCD_Puts(lcd_data);
	HAL_Delay(1000);
	memset(lcd_data, 0x00, sizeof(lcd_data));
}

void RTC_TimeShow_get()
{
	// LCD_Clear();
	RTC_TimeTypeDef getTime = {0};
	RTC_DateTypeDef getDate = {0};
	HAL_RTC_GetTime(&hrtc, &getTime, RTC_FORMAT_BIN);
	currRTCtimeMins = getTime.Hours * 60 + getTime.Minutes;
	HAL_RTC_GetDate(&hrtc, &getDate, RTC_FORMAT_BIN);
	//  dayIdx = 	getDate.WeekDay;
	//  prevWeekDay = dayIdx;
	//	LCD_Clear();
	// LCD_Gotoxy(0,0);
	// sprintf(lcd_data,"DATE: %02d/%02d/%4d", getDate.Date, getDate.Month, 2000+getDate.Year);
	// LCD_Puts(lcd_data);
	// sprintf(lcd_data,"TIME: %02d:%02d  %3s", getTime.Hours, getTime.Minutes, week[getDate.WeekDay-1]);
	// sprintf(lcd_data,"TIME: %02d:%02d:%02d", getTime.Hours, getTime.Minutes, getTime.Seconds);
	// LCD_Gotoxy(0,1);
	// LCD_Puts(lcd_data);
	// HAL_Delay(5000);
	// memset(lcd_data,0x00,sizeof(lcd_data));
}

void uct_EEPROM_FIXWRITE(int data)
{
	uint8_t *Data;
	uint8_t idx;

	if (data == PUMP_DATA)
	{
		Data = &pumpStatus;
		EEPROM_WriteNBytes(EP_PUMPSTATUS, Data, 1); // Error Flag

		Data = &pumpInitiationTimeSecs;
		EEPROM_WriteNBytes(EP_PUMPINITSECS, Data, 1); // Error Flag

		Data = (uint8_t *)&pumpRechargingTimeAbsMins;
		EEPROM_WriteNBytes(EP_RECHARGETIMEMINS, Data, 2); // Error Flag
	}
	else if (data == IRRIGATION_DATA)
	{
		uint16_t configured = 0xDCBA;

		Data = &weekdaysPgmA;
		EEPROM_WriteNBytes(EP_WEEKDAYS_PGMA, Data, 1); // Error Flag

		Data = &weekdaysPgmB;
		EEPROM_WriteNBytes(EP_WEEKDAYS_PGMB, Data, 1); // Error Flag

		Data = (uint8_t *)&irrigationMode[0];
		// EEPROM_WriteNBytes(EP_IRRG_MODE       , Data, 2); // Error Flag in ori
		for (idx = 0; idx < 2; idx++)
		{
			EEPROM_WriteByte(EP_IRRG_MODE + idx, Data[idx]); // Error Flag
		}

		Data = (uint8_t *)&masterSchdPumpSttimeMins[0];
		for (idx = 0; idx < 8; idx++)
		{
			EEPROM_WriteByte(EP_MASTER_PUMP_SCHD_STTIME + idx, Data[idx]); // Error Flag
		}

		// @@ ##
		Data = (uint8_t *)&masterSchdValveSttimeMins[0];
		for (idx = 0; idx < 96; idx++)
		{
			EEPROM_WriteByte(EP_MASTER_VALVE_SCHD_STTIME + idx, Data[idx]); // Error Flag
		}

		Data = (uint8_t *)&masterSchdValveDurationMins[0];
		for (idx = 0; idx < 96; idx++)
		{
			EEPROM_WriteByte(EP_MASTER_VALVE_SCHD_DUR + idx, Data[idx]); // Error Flag
		}

		Data = (uint8_t *)&get_total_v;				 // @@ ##
		EEPROM_WriteNBytes(EP_get_total_v, Data, 1);
		
		Data = (uint8_t *)&get_exp;				 // @@ ## is expension is enable(yes)(1) or not disable(no)(1)
		EEPROM_WriteNBytes(EP_get_exp, Data, 1);

		// ===
		Data = (uint8_t *)&masterSchdPreWetDurMins[0];
		for (idx = 0; idx < 96; idx++)
		{
			EEPROM_WriteByte(EP_MASTER_VALVE_PREWET_DUR + idx, Data[idx]); // Error Flag
		}

		Data = (uint8_t *)&masterSchdFertgDurMins[0];
		for (idx = 0; idx < 96; idx++)
		{
			EEPROM_WriteByte(EP_MASTER_VALVE_FERTG_DUR + idx, Data[idx]); // Error Flag
		}

		Data = (uint8_t *)&masterSchdFlushingDurMins[0];
		for (idx = 0; idx < 96; idx++)
		{
			EEPROM_WriteByte(EP_MASTER_VALVE_FLUSH_DUR + idx, Data[idx]); // Error Flag
		}

		Data = (uint8_t *)&valve_count;
		EEPROM_WriteNBytes(EP_VALVECNT, Data, 1); // Valve count

		Data = (uint8_t *)&effectiveFromToday;
		EEPROM_WriteNBytes(EP_EFFECTIVETODAY, Data, 1); // Valve count
														// changed by rishabh
		Data = (uint8_t *)&configured;
		EEPROM_WriteNBytes(EP_CONFIGURED, Data, 2); // Configured Flag

		ScheduleComplete = 0;
		Data = &ScheduleComplete;
		EEPROM_WriteNBytes(EP_SCHEDULE_COMPLETE, Data, 1); // Error Flag
	}
	
	else if (data == CLOUD_DATA)
	{
		Data = (uint8_t *)&server[0];
		for (idx = 0; idx < 16; idx++)
		{
			EEPROM_WriteByte(EP_server_IP + idx, Data[idx]); // Error Flag
		}
		
		Data = (uint8_t *)&apn[0];
		for (idx = 0; idx < 16; idx++)
		{
			EEPROM_WriteByte(EP_apn + idx, Data[idx]); // Error Flag
		}

	}
	
	
}

void uct_EEPROM_UPDATEDWRITE()
{
	uint8_t *Data;
	uint8_t idx;
	uint16_t configured = 0xDCBA;

	Data = (uint8_t *)&pendIrrgFlag;
	EEPROM_WriteNBytes(EP_PENDIRRG, Data, 1); // pendIrrgFlag Flag

	Data = &CurrSchedIdx;
	EEPROM_WriteNBytes(EP_CURRSCHIDX, Data, 1); // Error Flag

	// @@ ##
	Data = (uint8_t *)&todaySchdValveSttimeMins[0];
	for (idx = 0; idx < 96; idx++)
	{
		EEPROM_WriteByte(EP_TODAY_VALVE_SCHD_STTIME + idx, Data[idx]); // Error Flag
	}

	Data = (uint8_t *)&todaySchdValveDurationMins[0];
	for (idx = 0; idx < 96; idx++)
	{
		EEPROM_WriteByte(EP_TODAY_VALVE_SCHD_DUR + idx, Data[idx]); // Error Flag
	}

	// ===

	Data = (uint8_t *)&todaySchdPreWetDurMins[0];
	for (idx = 0; idx < 96; idx++)
	{
		EEPROM_WriteByte(EP_TODAY_VALVE_PREWET_DUR + idx, Data[idx]); // Error Flag
	}

	Data = (uint8_t *)&todaySchdFertgDurMins[0];
	for (idx = 0; idx < 96; idx++)
	{
		EEPROM_WriteByte(EP_TODAY_VALVE_FERTG_DUR + idx, Data[idx]); // Error Flag
	}

	Data = (uint8_t *)&todaySchdFlushingDurMins[0];
	for (idx = 0; idx < 96; idx++)
	{
		EEPROM_WriteByte(EP_TODAY_VALVE_FLUSH_DUR + idx, Data[idx]); // Error Flag
	}
}

void uct_EEPROM_READ()
{
	uint8_t idx;
	uint8_t *Data;
	Data = &errorFlag;
	EEPROM_ReadNBytes(EP_ERRORFLAG, Data, 1); // Error Flag

	Data = &pumpStatus;
	EEPROM_ReadNBytes(EP_PUMPSTATUS, Data, 1); // Error Flag

	Data = &pumpInitiationTimeSecs;
	EEPROM_ReadNBytes(EP_PUMPINITSECS, Data, 1); // Error Flag
	if (pumpInitiationTimeSecs > 90)
	{
		pumpInitiationTimeSecs = 5;
	}

	Data = (uint8_t *)&pumpRechargingTimeAbsMins;
	EEPROM_ReadNBytes(EP_RECHARGETIMEMINS, Data, 2); // Error Flag
	if (pumpRechargingTimeAbsMins > 1440)
	{
		pumpRechargingTimeAbsMins = 1;
	}

	Data = &weekdaysPgmA;
	EEPROM_ReadNBytes(EP_WEEKDAYS_PGMA, Data, 1); // Error Flag

	Data = (uint8_t *)&irrigationMode[0];
	// EEPROM_ReadNBytes(EP_IRRG_MODE       , Data, 2); // Error Flag
	for (idx = 0; idx < 2; idx++)
	{
		Data[idx] = EEPROM_ReadByte(EP_IRRG_MODE + idx); // Error Flag
	}

	Data = &CurrSchedIdx;
	EEPROM_ReadNBytes(EP_CURRSCHIDX, Data, 1); // Error Flag

	Data = &ScheduleComplete;
	EEPROM_ReadNBytes(EP_SCHEDULE_COMPLETE, Data, 1); // Error Flag

	Data = (uint8_t *)&valve_count;
	EEPROM_ReadNBytes(EP_VALVECNT, Data, 1); // Valve count

	Data = (uint8_t *)&effectiveFromToday;
	EEPROM_ReadNBytes(EP_EFFECTIVETODAY, Data, 1); // Valve count

	// change by sunny
	Data = (uint8_t *)&pendIrrgFlag;
	EEPROM_ReadNBytes(EP_PENDIRRG, Data, 1); // pendIrrgFlag Flag
	
	// ## if there pendIrrgFlag = 1, then ,this block decide the value of prevWeekDay and dayIdx ,, and for pendIrrgFlag = 1 ,, prevWeekDay (not_equal) to dayIdx
	if (pendIrrgFlag == 1)
	{
		currRTCtimeMins = currRTCtimeMins + 1440;

		prevWeekDay = (dayIdx + 7) - 1;

		if (prevWeekDay > 7)
		{
			prevWeekDay = prevWeekDay % 7;
		}

		dayIdx = (dayIdx + 7) - 1;

		if (dayIdx > 7)
		{
			dayIdx = dayIdx % 7;
		}
	}

	Data = (uint8_t *)&weekdaysPgmB;
	EEPROM_ReadNBytes(EP_WEEKDAYS_PGMB, Data, 1); // Valve count

	Data = (uint8_t *)&SecondRTdayIdx;
	EEPROM_ReadNBytes(EP_2ND_RT_DAY, Data, 1);

	Data = (uint8_t *)&SecondRTTimeInstanceMins;
	for (idx = 0; idx < 2; idx++)
	{
		Data[idx] = EEPROM_ReadByte(EP_2ND_RT_INSTMINS + idx); // Error Flag
	}


// === ##
	Data = (uint8_t *)&masterSchdPumpSttimeMins[0];
	for (idx = 0; idx < 8; idx++)
	{
		Data[idx] = EEPROM_ReadByte(EP_MASTER_PUMP_SCHD_STTIME + idx); // Error Flag
	}

	Data = (uint8_t *)&masterSchdValveSttimeMins[0];
	// EEPROM_ReadNBytes(EP_MASTER_VALVE_SCHD_STTIME, Data,32 ); // Error Flag
	for (idx = 0; idx < 96; idx++)
	{
		Data[idx] = EEPROM_ReadByte(EP_MASTER_VALVE_SCHD_STTIME + idx); // Error Flag
	}

	Data = (uint8_t *)&todaySchdValveSttimeMins[0];
	// EEPROM_ReadNBytes(EP_TODAY_VALVE_SCHD_STTIME, Data, 32); // Error Flag
	for (idx = 0; idx < 96; idx++)
	{
		Data[idx] = EEPROM_ReadByte(EP_TODAY_VALVE_SCHD_STTIME + idx); // Error Flag
	}

	Data = (uint8_t *)&masterSchdValveDurationMins[0];
	// EEPROM_ReadNBytes(EP_TODAY_VALVE_SCHD_DUR, Data, 32); // Error Flag
	for (idx = 0; idx < 96; idx++)
	{
		Data[idx] = EEPROM_ReadByte(EP_MASTER_VALVE_SCHD_DUR + idx); // Error Flag
	}

	Data = (uint8_t *)&get_total_v;				// @@ ## del it
	EEPROM_ReadNBytes(EP_get_total_v, Data, 1); // Error Flag
	
	Data = (uint8_t *)&get_exp;				 // @@ ## is expension is enable(yes)(1) or not disable(no)(1)
	EEPROM_ReadNBytes(EP_get_exp, Data, 1);

	Data = (uint8_t *)&masterSchdPreWetDurMins[0];
	// EEPROM_ReadNBytes(EP_TODAY_VALVE_PREWET_DUR, Data, 32); // Error Flag
	for (idx = 0; idx < 96; idx++)
	{
		Data[idx] = EEPROM_ReadByte(EP_MASTER_VALVE_PREWET_DUR + idx); // Error Flag
	}

	Data = (uint8_t *)&masterSchdFertgDurMins[0];
	// EEPROM_ReadNBytes(EP_TODAY_VALVE_FERTG_DUR, Data, 32); // Error Flag
	for (idx = 0; idx < 96; idx++)
	{
		Data[idx] = EEPROM_ReadByte(EP_MASTER_VALVE_FERTG_DUR + idx); // Error Flag
	}

	Data = (uint8_t *)&masterSchdFlushingDurMins[0];
	// EEPROM_ReadNBytes(EP_TODAY_VALVE_FLUSH_DUR, Data, 32); // Error Flag
	for (idx = 0; idx < 96; idx++)
	{
		Data[idx] = EEPROM_ReadByte(EP_MASTER_VALVE_FLUSH_DUR + idx); // Error Flag
	}

// === ## 
	Data = (uint8_t *)&todaySchdValveDurationMins[0];
	// EEPROM_ReadNBytes(EP_TODAY_VALVE_SCHD_DUR, Data, 32); // Error Flag
	for (idx = 0; idx < 96; idx++)
	{
		Data[idx] = EEPROM_ReadByte(EP_TODAY_VALVE_SCHD_DUR + idx); // Error Flag
	}

	Data = (uint8_t *)&todaySchdPreWetDurMins[0];
	// EEPROM_ReadNBytes(EP_TODAY_VALVE_PREWET_DUR, Data, 32); // Error Flag
	for (idx = 0; idx < 96; idx++)
	{
		Data[idx] = EEPROM_ReadByte(EP_TODAY_VALVE_PREWET_DUR + idx); // Error Flag
	}

	Data = (uint8_t *)&todaySchdFertgDurMins[0];
	// EEPROM_ReadNBytes(EP_TODAY_VALVE_FERTG_DUR, Data, 32); // Error Flag
	for (idx = 0; idx < 96; idx++)
	{
		Data[idx] = EEPROM_ReadByte(EP_TODAY_VALVE_FERTG_DUR + idx); // Error Flag
	}

	Data = (uint8_t *)&todaySchdFlushingDurMins[0];
	// EEPROM_ReadNBytes(EP_TODAY_VALVE_FLUSH_DUR, Data, 32); // Error Flag
	for (idx = 0; idx < 96; idx++)
	{
		Data[idx] = EEPROM_ReadByte(EP_TODAY_VALVE_FLUSH_DUR + idx); // Error Flag
	}

	// serverIP read
	Data = (uint8_t *)&server[0];
	for (idx = 0; idx < 16; idx++)
	{
		Data[idx] = EEPROM_ReadByte(EP_server_IP + idx); // Error Flag
	}
	
	// apn read
	Data = (uint8_t *)&apn[0];
	for (idx = 0; idx < 16; idx++)
	{
		Data[idx] = EEPROM_ReadByte(EP_apn + idx); // Error Flag
	}

}

void printPgmStatus()
{
	uint16_t sthrs, stmins;
	uint8_t curridx, pgmidx;
	uint8_t *Data;
	uint16_t checkConfigured = 0;

	Data = (uint8_t *)&checkConfigured;
	EEPROM_ReadNBytes(EP_CONFIGURED, Data, 2); // Configured Flag

	LCD_Clear();
	LCD_Gotoxy(0, 0);

	if (checkConfigured == 0xDCBA) // if (checkConfigured == 0x0694)
	{
		uct_EEPROM_READ();

		low_flow_status = HAL_GPIO_ReadPin(LOW_FLOW_GPIO_Port, LOW_FLOW_Pin);
		HAL_Delay(1000);
		low_flow_status = HAL_GPIO_ReadPin(LOW_FLOW_GPIO_Port, LOW_FLOW_Pin);
		if (low_flow_status == 0)
		{
			halt_alloperation = 0;
			if ((errorFlag & (1 << LOW_FLOW_2ND_RT)) != 0)
			{
				uint8_t Data = 0;
				EEPROM_WriteNBytes(EP_ERRORFLAG, &Data, 1); // Error Flag
				errorFlag = 0;
			}
		}

		else if ((errorFlag & (1 << LOW_FLOW_2ND_RT)) != 0)
		{
			halt_alloperation = 1;
		}
		weekdays = weekdaysPgmA | weekdaysPgmB;

		if (weekdays == 0)
		{
			LCD_Gotoxy(0, 1);
			LCD_Puts("DAY NOT PROGRAM ");
			daynotconfigured = 1;
			HAL_Delay(500);
		}
		else
		{
			for (int id = 0; id < valve_count; id++)
			{
				todaySchdPreWetSttimeMins[id] = todaySchdValveSttimeMins[id];
				todaySchdFertgSttimeMins[id] = todaySchdPreWetSttimeMins[id] + todaySchdPreWetDurMins[id];
				todaySchdFlushingSttimeMins[id] = todaySchdFertgSttimeMins[id] + todaySchdFertgDurMins[id];
			}

			if (todaySchdValveSttimeMins[CurrSchedIdx] != 0)
			{
				sthrs = todaySchdValveSttimeMins[CurrSchedIdx] / 60;
				stmins = todaySchdValveSttimeMins[CurrSchedIdx] - sthrs * 60;
				curridx = CurrSchedIdx % 12;
				// pgmidx = CurrSchedIdx >> 3;
				pgmidx = CurrSchedIdx / 24;

				if (pgmidx == 0 && irrigationMode[pgmidx] != 2)
					sprintf(lcd_data, "PGM=A  V=%d IRRG", (curridx + 1));
				if (pgmidx == 0 && irrigationMode[pgmidx] == 2)
					sprintf(lcd_data, "PGM=A  V=%d FERT", (curridx + 1));
				if (pgmidx == 1 && irrigationMode[pgmidx] == 2)
					sprintf(lcd_data, "PGM=B  V=%d FERT", (curridx + 1));
				if (pgmidx == 1 && irrigationMode[pgmidx] != 2)
					sprintf(lcd_data, "PGM=B  V=%d IRRG", (curridx + 1));

				LCD_Puts(lcd_data);
				LCD_Gotoxy(0, 1);
				notconfigured = 0;
				// sprintf(lcd_data,"ST=%02d:%02d DUR%03dm",sthrs,stmins,todaySchdValveDurationMins[CurrSchedIdx]);
				//				if(todaySchdValveSttimeMins[CurrSchedIdx]<1440)
				//				{
				// sprintf(lcd_data,"START = %02d:%02d         ",sthrs ,stmins);
				sprintf(lcd_data, "START = %02d:%02d         ", sthrs % 24, stmins);
				LCD_Puts(lcd_data);
				//				}
			}
			else
			{
				LCD_Puts("PROGRAMMING     ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("NOT DONE  ");
			}
			HAL_Delay(500);
		}
	}
	else
	{
		//		LCD_Gotoxy(0,1);
		//		LCD_Puts("       ");
		notconfigured = 1;
	}
	HAL_Delay(500);
}

void printErrStatus()
{
	uint8_t idx = 0;
	// for (idx=0; idx<4; idx+=2)
	{
		// ## RAIN SHUT OFF, error display, here errorFlag=0x01, idx=0x00, then becomes if(1)
		if ((errorFlag >> idx) & 0x1)
		{
			// LCD_Clear();
			LCD_Gotoxy(0, 1);
			sprintf(lcd_data, "%s", errorInfoArr[idx]);
			LCD_Puts(lcd_data);
			HAL_Delay(500);
		}
		// ## MAINS OFF, error display, here errorFlag=0x02, idx=0x00, then becomes if(1)
		if ((errorFlag >> (idx + 1)) & 0x1)
		{
			LCD_Gotoxy(0, 1);
			sprintf(lcd_data, "%s", errorInfoArr[idx + 1]);
			LCD_Puts(lcd_data);
			HAL_Delay(500);
		}
	
	}
	
	//	 if ((errorFlag >> 4) & 0x1)
	//	 {
	//		 //LCD_Clear();
	//		 LCD_Gotoxy(0,1);
	//		 sprintf(lcd_data,"%s", errorInfoArr[4]);
	//		 LCD_Puts(lcd_data);
	//		 HAL_Delay(500);
	//	 }

	//	 if ((errorFlag >> 5) & 0x1)
	//	 {
	//		 //LCD_Clear();
	//		 LCD_Gotoxy(0,1);
	//		 sprintf(lcd_data,"%s", errorInfoArr[5]);
	//		 LCD_Puts(lcd_data);
	//		 HAL_Delay(500);
	//	 }
}

void displayConfig()
{
	//   		uint8_t pgmidx,d_c = 1;
	//		int a = irrigationMode[pgmidx];
	//	  pgmidx = CurrSchedIdx >>3;
	//	  sw1 = 0;
	//
	//		while(d_c==1)
	//		{
	//			if((sw1==1) && input_timeout<1000)
	//			{
	//				sprintf(msg,"Valve:%d",CurrSchedIdx);
	//				LCD_Gotoxy(0,0);
	//
	//			}
	//		}

	uint16_t sthrs, stmins;
	uint8_t curridx, pgmidx, idx, week_show_flag;
	uint8_t *Data, i, j = 1, fl = 1, fl1 = 1, fl2 = 1;
	uint16_t checkConfigured = 0;

	Data = (uint8_t *)&checkConfigured;
	EEPROM_ReadNBytes(EP_CONFIGURED, Data, 2); // Configured Flag

	curridx = CurrSchedIdx % 12; // @@ ## for val no. currently runing, CurrSchedIdx range from 0 to 47
	pgmidx = CurrSchedIdx / 24;  // this pgmidx can be 0 or 1 (nothing else), 0 for pgA(0--23), 1 for pgB(24--47) // @@ ##
	idx = CurrSchedIdx;

	LCD_Clear();
	LCD_Gotoxy(0, 0);

	disp = 1;
	//	if(Run_State == 1)
	//	stopSpecialSequence();
	sw2 = 1;
	


	if ((sw2 == SW_START) && (input_timeout < SW_DELAY))
	{
		LCD_Clear();

		if (pumpStatus == 0)
		{
			LCD_Gotoxy(0, 0);
			LCD_Puts("PUMP:OFF INIT:0s");
			LCD_Gotoxy(0, 1);
			LCD_Puts("RECHARGE- 00:00 ");
			HAL_Delay(500);
		}
		else // if (pumpStatus == 1)
		{
			LCD_Gotoxy(0, 0);
			sprintf(lcd_data, "PUMP:ON INIT:%ds", pumpInitiationTimeSecs);
			LCD_Puts(lcd_data);
			LCD_Gotoxy(0, 1);
			sprintf(lcd_data, "RECHARGE- %02d:%02d ", pumpRechargingTimeAbsMins / 60, pumpRechargingTimeAbsMins % 60);
			LCD_Puts(lcd_data);
			HAL_Delay(1500);
		}
		//        else
		//				{
		//					LCD_Gotoxy(0,0);
		//					LCD_Puts("PUMP              ");
		//					LCD_Gotoxy(0,1);
		//					LCD_Puts("NOT CONFIGURED   ");
		//					HAL_Delay(500);
		//				}
	}
	if (checkConfigured == 0xDCBA)
	{
		// uct_EEPROM_READ();
		HAL_Delay(100);
		for (int id = 0; id < valve_count; id++)
		{
			todaySchdPreWetSttimeMins[id] = todaySchdValveSttimeMins[id];
			todaySchdFertgSttimeMins[id] = todaySchdPreWetSttimeMins[id] + todaySchdPreWetDurMins[id];
			todaySchdFlushingSttimeMins[id] = todaySchdFertgSttimeMins[id] + todaySchdFertgDurMins[id];
		}

		if (todaySchdValveSttimeMins[idx] != 0)
		{
			sthrs = todaySchdValveSttimeMins[idx] / 60;
			stmins = todaySchdValveSttimeMins[idx] - sthrs * 60;

			while (disp == 1)
			{


				// ======pgA displayConfig
				if (masterSchdPumpSttimeMins[0] != 0)
				{

					//j = 1; // for display start time 1 --> st-1 ## @@

					// for start st-1 pgA
					if ((sw2 == SW_START + 1) && (input_timeout < SW_DELAY))
					{
						curridx = 0;
						idx = 0;
						pgmidx = 0;
						LCD_Clear();
						LCD_Gotoxy(0, 0);

						if (pgmidx == 0 && irrigationMode[pgmidx] != 2)
							sprintf(lcd_data, "PGM=A ST%d  IRRG", j);
						if (pgmidx == 0 && irrigationMode[pgmidx] == 2)
							sprintf(lcd_data, "PGM=A ST%d  FERT", j);

						LCD_Puts(lcd_data);
						LCD_Gotoxy(0, 1);

						sprintf(lcd_data, "START = %02d:%02d         ", masterSchdPumpSttimeMins[0] / 60, masterSchdPumpSttimeMins[0] % 60);
						LCD_Puts(lcd_data);
						HAL_Delay(500);
					}

					// for start st-2 pgA
					if ((sw2 == SW_START + 2) && (input_timeout < SW_DELAY))
					{

						pgmidx = 0;
						idx = 12;
						displayConfig_flag = 1;

						LCD_Clear();
						LCD_Gotoxy(0, 0);

						if (fl == 1)
						{
							sthrs = todaySchdValveSttimeMins[idx] / 60;
							stmins = todaySchdValveSttimeMins[idx] - sthrs * 60;
						}
						fl = 0;

						if (pgmidx == 0 && irrigationMode[pgmidx] != 2)
							sprintf(lcd_data, "PGM=A ST%d  IRRG", j + 1);
						if (pgmidx == 0 && irrigationMode[pgmidx] == 2)
							sprintf(lcd_data, "PGM=A ST%d  FERT", j + 1);

						LCD_Puts(lcd_data);
						LCD_Gotoxy(0, 1);
						sprintf(lcd_data, "START = %02d:%02d         ", masterSchdPumpSttimeMins[1] / 60, masterSchdPumpSttimeMins[1] % 60);
						LCD_Puts(lcd_data);
						HAL_Delay(500);
					
					}

	


					// if exp no,, for default, if select 4 , for pgA
					if (get_total_v == 4)
					{
					g4:
						if ((sw2 == SW_START + 3) && (input_timeout < SW_DELAY))
						{
							
							if (displayConfig_flag ==1)
							{
								curridx = 0;
								pgmidx = 0;
								idx = 0;
								displayConfig_flag =0;
							}
		
							
							LCD_Clear();
							LCD_Gotoxy(0, 0);

							if (pgmidx == 0 && irrigationMode[pgmidx] != 2)
								sprintf(lcd_data, "PGM=A         IRRG");
							if (pgmidx == 0 && irrigationMode[pgmidx] == 2)
								sprintf(lcd_data, "PGM=A         FERT");

							LCD_Puts(lcd_data);
							LCD_Gotoxy(0, 1);
							sprintf(lcd_data, "Valve%d Dur=%d m       ", curridx + 1, masterSchdValveDurationMins[idx]);
							LCD_Puts(lcd_data);
							HAL_Delay(500);
							if (irrigationMode[pgmidx] == 2 && (masterSchdValveDurationMins[idx] > 0))
							{
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "V%d Dur=%d m PRE", curridx + 1, masterSchdPreWetDurMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(1000);
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "V%d Dur=%d m FER", curridx + 1, masterSchdFertgDurMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(1000);
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "V%d Dur=%d m FLS", curridx + 1, masterSchdFlushingDurMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(1000);
							}
						}


						if ((sw2 == (SW_START + 4) && (input_timeout < SW_DELAY)))
						{
							// MenuDoneCnt = SW_PGM_SEL_ST+1;

							curridx = curridx + 1; // @@ ##
							idx = idx + 1;

						
							if (curridx == 4)
							{
								week_show_flag = 1;
								sw2 = SW_START + 5;
							}
							else
							{
								sw2 = SW_START + 3;
								goto g4;
							}	
						} // end if (sw + 4)

					} // end for valve 4
				
					
					// if exp yes,, if select 5 , for pgA
					if (get_total_v == 5)
					{
					g5:
						if ((sw2 == SW_START + 3) && (input_timeout < SW_DELAY))
						{
							
							if (displayConfig_flag ==1)
							{
								curridx = 0;
								pgmidx = 0;
								idx = 0;
								displayConfig_flag =0;
							}
		
							
							LCD_Clear();
							LCD_Gotoxy(0, 0);

							if (pgmidx == 0 && irrigationMode[pgmidx] != 2)
								sprintf(lcd_data, "PGM=A         IRRG");
							if (pgmidx == 0 && irrigationMode[pgmidx] == 2)
								sprintf(lcd_data, "PGM=A         FERT");

							LCD_Puts(lcd_data);
							LCD_Gotoxy(0, 1);
							sprintf(lcd_data, "Valve%d Dur=%d m       ", curridx + 1, masterSchdValveDurationMins[idx]);
							LCD_Puts(lcd_data);
							HAL_Delay(500);
							if (irrigationMode[pgmidx] == 2 && (masterSchdValveDurationMins[idx] > 0))
							{
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "V%d Dur=%d m PRE", curridx + 1, masterSchdPreWetDurMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(1000);
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "V%d Dur=%d m FER", curridx + 1, masterSchdFertgDurMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(1000);
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "V%d Dur=%d m FLS", curridx + 1, masterSchdFlushingDurMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(1000);
							}
						}


						if ((sw2 == (SW_START + 4) && (input_timeout < SW_DELAY)))
						{
							// MenuDoneCnt = SW_PGM_SEL_ST+1;

							curridx = curridx + 1; // @@ ##
							idx = idx + 1;

						
							if (curridx == 5)
							{
								week_show_flag = 1;
								sw2 = SW_START + 5;
							}
							else
							{
								sw2 = SW_START + 3;
								goto g5;
							}	
						} // end if (sw + 4)

					} // end for valve 5
				

					// if exp yes,, if select 6 , for pgA
					if (get_total_v == 6)
					{
					g6:
						if ((sw2 == SW_START + 3) && (input_timeout < SW_DELAY))
						{
							
							if (displayConfig_flag ==1)
							{
								curridx = 0;
								pgmidx = 0;
								idx = 0;
								displayConfig_flag =0;
							}
		
							
							LCD_Clear();
							LCD_Gotoxy(0, 0);

							if (pgmidx == 0 && irrigationMode[pgmidx] != 2)
								sprintf(lcd_data, "PGM=A         IRRG");
							if (pgmidx == 0 && irrigationMode[pgmidx] == 2)
								sprintf(lcd_data, "PGM=A         FERT");

							LCD_Puts(lcd_data);
							LCD_Gotoxy(0, 1);
							sprintf(lcd_data, "Valve%d Dur=%d m       ", curridx + 1, masterSchdValveDurationMins[idx]);
							LCD_Puts(lcd_data);
							HAL_Delay(500);
							if (irrigationMode[pgmidx] == 2 && (masterSchdValveDurationMins[idx] > 0))
							{
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "V%d Dur=%d m PRE", curridx + 1, masterSchdPreWetDurMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(1000);
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "V%d Dur=%d m FER", curridx + 1, masterSchdFertgDurMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(1000);
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "V%d Dur=%d m FLS", curridx + 1, masterSchdFlushingDurMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(1000);
							}
						}


						if ((sw2 == (SW_START + 4) && (input_timeout < SW_DELAY)))
						{
							// MenuDoneCnt = SW_PGM_SEL_ST+1;

							curridx = curridx + 1; // @@ ##
							idx = idx + 1;

						
							if (curridx == 6)
							{
								week_show_flag = 1;
								sw2 = SW_START + 5;
							}
							else
							{
								sw2 = SW_START + 3;
								goto g6;
							}	
						} // end if (sw + 4)

					} // end for valve 6
				
					

					// if exp yes,, if select 7 , for pgA
					if (get_total_v == 7)
					{
					g7:
						if ((sw2 == SW_START + 3) && (input_timeout < SW_DELAY))
						{
							
							if (displayConfig_flag ==1)
							{
								curridx = 0;
								pgmidx = 0;
								idx = 0;
								displayConfig_flag =0;
							}
		
							
							LCD_Clear();
							LCD_Gotoxy(0, 0);

							if (pgmidx == 0 && irrigationMode[pgmidx] != 2)
								sprintf(lcd_data, "PGM=A         IRRG");
							if (pgmidx == 0 && irrigationMode[pgmidx] == 2)
								sprintf(lcd_data, "PGM=A         FERT");

							LCD_Puts(lcd_data);
							LCD_Gotoxy(0, 1);
							sprintf(lcd_data, "Valve%d Dur=%d m       ", curridx + 1, masterSchdValveDurationMins[idx]);
							LCD_Puts(lcd_data);
							HAL_Delay(500);
							if (irrigationMode[pgmidx] == 2 && (masterSchdValveDurationMins[idx] > 0))
							{
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "V%d Dur=%d m PRE", curridx + 1, masterSchdPreWetDurMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(1000);
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "V%d Dur=%d m FER", curridx + 1, masterSchdFertgDurMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(1000);
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "V%d Dur=%d m FLS", curridx + 1, masterSchdFlushingDurMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(1000);
							}
						}


						if ((sw2 == (SW_START + 4) && (input_timeout < SW_DELAY)))
						{
							// MenuDoneCnt = SW_PGM_SEL_ST+1;

							curridx = curridx + 1; // @@ ##
							idx = idx + 1;

						
							if (curridx == 7)
							{
								week_show_flag = 1;
								sw2 = SW_START + 5;
							}
							else
							{
								sw2 = SW_START + 3;
								goto g7;
							}	
						} // end if (sw + 4)

					} // end for valve 7
				
					

					// if exp yes,, if select 8 , for pgA
					if (get_total_v == 8)
					{
					g8:
						if ((sw2 == SW_START + 3) && (input_timeout < SW_DELAY))
						{
							
							if (displayConfig_flag ==1)
							{
								curridx = 0;
								pgmidx = 0;
								idx = 0;
								displayConfig_flag =0;
							}
		
							
							LCD_Clear();
							LCD_Gotoxy(0, 0);

							if (pgmidx == 0 && irrigationMode[pgmidx] != 2)
								sprintf(lcd_data, "PGM=A         IRRG");
							if (pgmidx == 0 && irrigationMode[pgmidx] == 2)
								sprintf(lcd_data, "PGM=A         FERT");

							LCD_Puts(lcd_data);
							LCD_Gotoxy(0, 1);
							sprintf(lcd_data, "Valve%d Dur=%d m       ", curridx + 1, masterSchdValveDurationMins[idx]);
							LCD_Puts(lcd_data);
							HAL_Delay(500);
							if (irrigationMode[pgmidx] == 2 && (masterSchdValveDurationMins[idx] > 0))
							{
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "V%d Dur=%d m PRE", curridx + 1, masterSchdPreWetDurMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(1000);
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "V%d Dur=%d m FER", curridx + 1, masterSchdFertgDurMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(1000);
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "V%d Dur=%d m FLS", curridx + 1, masterSchdFlushingDurMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(1000);
							}
						}


						if ((sw2 == (SW_START + 4) && (input_timeout < SW_DELAY)))
						{
							// MenuDoneCnt = SW_PGM_SEL_ST+1;

							curridx = curridx + 1; // @@ ##
							idx = idx + 1;

						
							if (curridx == 8)
							{
								week_show_flag = 1;
								sw2 = SW_START + 5;
							}
							else
							{
								sw2 = SW_START + 3;
								goto g8;
							}	
						} // end if (sw + 4)

					} // end for valve 8
				
					

					// if exp yes,, if select 9 , for pgA
					if (get_total_v == 9)
					{
					g9:
						if ((sw2 == SW_START + 3) && (input_timeout < SW_DELAY))
						{
							
							if (displayConfig_flag ==1)
							{
								curridx = 0;
								pgmidx = 0;
								idx = 0;
								displayConfig_flag =0;
							}
		
							
							LCD_Clear();
							LCD_Gotoxy(0, 0);

							if (pgmidx == 0 && irrigationMode[pgmidx] != 2)
								sprintf(lcd_data, "PGM=A         IRRG");
							if (pgmidx == 0 && irrigationMode[pgmidx] == 2)
								sprintf(lcd_data, "PGM=A         FERT");

							LCD_Puts(lcd_data);
							LCD_Gotoxy(0, 1);
							sprintf(lcd_data, "Valve%d Dur=%d m       ", curridx + 1, masterSchdValveDurationMins[idx]);
							LCD_Puts(lcd_data);
							HAL_Delay(500);
							if (irrigationMode[pgmidx] == 2 && (masterSchdValveDurationMins[idx] > 0))
							{
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "V%d Dur=%d m PRE", curridx + 1, masterSchdPreWetDurMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(1000);
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "V%d Dur=%d m FER", curridx + 1, masterSchdFertgDurMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(1000);
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "V%d Dur=%d m FLS", curridx + 1, masterSchdFlushingDurMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(1000);
							}
						}


						if ((sw2 == (SW_START + 4) && (input_timeout < SW_DELAY)))
						{
							// MenuDoneCnt = SW_PGM_SEL_ST+1;

							curridx = curridx + 1; // @@ ##
							idx = idx + 1;

						
							if (curridx == 9)
							{
								week_show_flag = 1;
								sw2 = SW_START + 5;
							}
							else
							{
								sw2 = SW_START + 3;
								goto g9;
							}	
						} // end if (sw + 4)

					} // end for valve 9
				
					

					// if exp yes,, if select 10 , for pgA
					if (get_total_v == 10)
					{
					g10:
						if ((sw2 == SW_START + 3) && (input_timeout < SW_DELAY))
						{
							
							if (displayConfig_flag ==1)
							{
								curridx = 0;
								pgmidx = 0;
								idx = 0;
								displayConfig_flag =0;
							}
		
							
							LCD_Clear();
							LCD_Gotoxy(0, 0);

							if (pgmidx == 0 && irrigationMode[pgmidx] != 2)
								sprintf(lcd_data, "PGM=A       IRRG", j);
							if (pgmidx == 0 && irrigationMode[pgmidx] == 2)
								sprintf(lcd_data, "PGM=A       FERT", j);

							LCD_Puts(lcd_data);
							LCD_Gotoxy(0, 1);
							sprintf(lcd_data, "Valve%d Dur=%d m       ", curridx + 1, masterSchdValveDurationMins[idx]);
							LCD_Puts(lcd_data);
							HAL_Delay(500);
							if (irrigationMode[pgmidx] == 2 && (masterSchdValveDurationMins[idx] > 0))
							{
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "V%d Dur=%d m PRE", curridx + 1, masterSchdPreWetDurMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(1000);
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "V%d Dur=%d m FER", curridx + 1, masterSchdFertgDurMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(1000);
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "V%d Dur=%d m FLS", curridx + 1, masterSchdFlushingDurMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(1000);
							}
						}


						if ((sw2 == (SW_START + 4) && (input_timeout < SW_DELAY)))
						{
							// MenuDoneCnt = SW_PGM_SEL_ST+1;

							curridx = curridx + 1; // @@ ##
							idx = idx + 1;

						
							if (curridx == 10)
							{
								week_show_flag = 1;
								sw2 = SW_START + 5;
							}
							else
							{
								sw2 = SW_START + 3;
								goto g10;
							}	
						} // end if (sw + 4)

					} // end for valve 10
				
					

					// if exp yes,, if select 11 , for pgA
					if (get_total_v == 11)
					{
					g11:
						if ((sw2 == SW_START + 3) && (input_timeout < SW_DELAY))
						{
							
							if (displayConfig_flag ==1)
							{
								curridx = 0;
								pgmidx = 0;
								idx = 0;
								displayConfig_flag =0;
							}
		
							
							LCD_Clear();
							LCD_Gotoxy(0, 0);

							if (pgmidx == 0 && irrigationMode[pgmidx] != 2)
								sprintf(lcd_data, "PGM=A       IRRG", j);
							if (pgmidx == 0 && irrigationMode[pgmidx] == 2)
								sprintf(lcd_data, "PGM=A       FERT", j);

							LCD_Puts(lcd_data);
							LCD_Gotoxy(0, 1);
							sprintf(lcd_data, "Valve%d Dur=%d m       ", curridx + 1, masterSchdValveDurationMins[idx]);
							LCD_Puts(lcd_data);
							HAL_Delay(500);
							if (irrigationMode[pgmidx] == 2 && (masterSchdValveDurationMins[idx] > 0))
							{
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "V%d Dur=%d m PRE", curridx + 1, masterSchdPreWetDurMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(1000);
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "V%d Dur=%d m FER", curridx + 1, masterSchdFertgDurMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(1000);
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "V%d Dur=%d m FLS", curridx + 1, masterSchdFlushingDurMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(1000);
							}
						}


						if ((sw2 == (SW_START + 4) && (input_timeout < SW_DELAY)))
						{
							// MenuDoneCnt = SW_PGM_SEL_ST+1;

							curridx = curridx + 1; // @@ ##
							idx = idx + 1;

						
							if (curridx == 11)
							{
								week_show_flag = 1;
								sw2 = SW_START + 5;
							}
							else
							{
								sw2 = SW_START + 3;
								goto g11;
							}	
						} // end if (sw + 4)

					} // end for valve 11
				


					// if exp yes,, if select 12 , for pgA
					if (get_total_v == 12)
					{
					g12:
						if ((sw2 == SW_START + 3) && (input_timeout < SW_DELAY))
						{
							
							if (displayConfig_flag ==1)
							{
								curridx = 0;
								pgmidx = 0;
								idx = 0;
								displayConfig_flag =0;
							}
		
							
							LCD_Clear();
							LCD_Gotoxy(0, 0);

							if (pgmidx == 0 && irrigationMode[pgmidx] != 2)
								sprintf(lcd_data, "PGM=A         IRRG");
							if (pgmidx == 0 && irrigationMode[pgmidx] == 2)
								sprintf(lcd_data, "PGM=A         FERT");

							LCD_Puts(lcd_data);
							LCD_Gotoxy(0, 1);
							sprintf(lcd_data, "Valve%d Dur=%d m       ", curridx + 1, masterSchdValveDurationMins[idx]);
							LCD_Puts(lcd_data);
							HAL_Delay(500);
							if (irrigationMode[pgmidx] == 2 && (masterSchdValveDurationMins[idx] > 0))
							{
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "V%d Dur=%d m PRE", curridx + 1, masterSchdPreWetDurMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(1000);
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "V%d Dur=%d m FER", curridx + 1, masterSchdFertgDurMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(1000);
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "V%d Dur=%d m FLS", curridx + 1, masterSchdFlushingDurMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(1000);
							}
						}


						if ((sw2 == (SW_START + 4) && (input_timeout < SW_DELAY)))
						{
							// MenuDoneCnt = SW_PGM_SEL_ST+1;

							curridx = curridx + 1; // @@ ##
							idx = idx + 1;

						
							if (curridx == 12)
							{
								week_show_flag = 1;
								sw2 = SW_START + 5;
							}
							else
							{
								sw2 = SW_START + 3;
								goto g12;
							}	
						} // end if (sw + 4)

					} // end for valve 12
				
					
					


					// for pgA
					if (week_show_flag == 1)
					{

						if ((sw2 == (SW_START + 5) && (input_timeout < SW_DELAY)))
						{

							LCD_Clear();
							LCD_Gotoxy(0, 0);

							LCD_Puts("PGM=A ");

							LCD_Gotoxy(0, 1);
							if ((weekdaysPgmA & 0x01) == 1)
							{
								LCD_Puts("ALL DAYS     ");
							}
							else
							{
								// for(int dayi=1; dayi<8; dayi++)
								{
									LCD_Gotoxy(0, 1);
									if (((weekdaysPgmA >> 1) & 0x01) == 1)
										LCD_Puts("M               ");
									LCD_Gotoxy(2, 1);
									if (((weekdaysPgmA >> 2) & 0x01) == 1)
										LCD_Puts("T               ");
									LCD_Gotoxy(4, 1);
									if (((weekdaysPgmA >> 3) & 0x01) == 1)
										LCD_Puts("W               ");
									LCD_Gotoxy(6, 1);
									if (((weekdaysPgmA >> 4) & 0x01) == 1)
										LCD_Puts("Th               ");
									LCD_Gotoxy(9, 1);
									if (((weekdaysPgmA >> 5) & 0x01) == 1)
										LCD_Puts("F               ");
									LCD_Gotoxy(11, 1);
									if (((weekdaysPgmA >> 6) & 0x01) == 1)
										LCD_Puts("Sa               ");
									LCD_Gotoxy(14, 1);
									if (((weekdaysPgmA >> 7) & 0x01) == 1)
										LCD_Puts("Su               ");
								}
							}
							HAL_Delay(500);
										
							
							if ((sw2 == (SW_START + 6) && (input_timeout < SW_DELAY)))
							{
								pgmidx = 1; // to --> go to pgB
								sw2 = SW_START + 1; // go to pgB
								week_show_flag = 0;
								//HAL_Delay(1000);
								//break;
							}

						}

					}
			

				
				} // end pgA disconfig
				else
				{
					LCD_Clear();
					LCD_Puts("PGA");
					LCD_Gotoxy(0, 1);
					LCD_Puts("NOT PROGRAMMED  ");
					HAL_Delay(1000);
					pgmidx = 1; // to --> go to pgB
					sw2 = SW_START + 1; // go to pgB
				}

				// ======pgB displayConfig
				while (pgmidx == 1)
				{
				
					if ((masterSchdPumpSttimeMins[2] != 0))
					{
												
						// start time 1 for pgB // @@ ##
						if ((sw2 == SW_START + 1) && (input_timeout < SW_DELAY))
						{
							
							curridx = 0;
							idx = 0;
							pgmidx = 1;

							LCD_Clear();
							LCD_Gotoxy(0, 0);

							if (pgmidx == 1 && irrigationMode[pgmidx] != 2)
								sprintf(lcd_data, "PGM=B ST%d  IRRG", j);
							if (pgmidx == 1 && irrigationMode[pgmidx] == 2)
								sprintf(lcd_data, "PGM=B ST%d  FERT", j);

							LCD_Puts(lcd_data);
							LCD_Gotoxy(0, 1);

							sprintf(lcd_data, "START = %02d:%02d         ", masterSchdPumpSttimeMins[2] / 60, masterSchdPumpSttimeMins[2] % 60);
							LCD_Puts(lcd_data);
							HAL_Delay(500);
						}

						// start time 2 for pgB // @@ ##
						if ((sw2 == SW_START + 2) && (input_timeout < SW_DELAY))
						{
							pgmidx = 1;
							idx = 12;
							displayConfig_flag = 1;

							LCD_Clear();
							LCD_Gotoxy(0, 0);

							if (fl == 1)
							{
								sthrs = todaySchdValveSttimeMins[idx] / 60;
								stmins = todaySchdValveSttimeMins[idx] - sthrs * 60;
							}
							fl = 0;

							if (pgmidx == 1 && irrigationMode[pgmidx] != 2)
								sprintf(lcd_data, "PGM=B ST%d  IRRG", j + 1);
							if (pgmidx == 1 && irrigationMode[pgmidx] == 2)
								sprintf(lcd_data, "PGM=B ST%d  FERT", j + 1);

							LCD_Puts(lcd_data);
							LCD_Gotoxy(0, 1);
							sprintf(lcd_data, "START = %02d:%02d         ", masterSchdPumpSttimeMins[3] / 60, masterSchdPumpSttimeMins[3] % 60);
							LCD_Puts(lcd_data);
							HAL_Delay(500);
						}



						// if exp no,, for default, if select 4 , for pgB
						if (get_total_v == 4)
						{
						pb4:
							if ((sw2 == SW_START + 3) && (input_timeout < SW_DELAY))
							{
								
								if (displayConfig_flag ==1)
								{
									curridx = 0;
									pgmidx = 1;
									idx = 24;
									displayConfig_flag =0;
								}
			
								
								LCD_Clear();
								LCD_Gotoxy(0, 0);

								if (pgmidx == 1 && irrigationMode[pgmidx] != 2)
									sprintf(lcd_data, "PGM=B         IRRG");
								if (pgmidx == 1 && irrigationMode[pgmidx] == 2)
									sprintf(lcd_data, "PGM=B         FERT");

								LCD_Puts(lcd_data);
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "Valve%d Dur=%d m       ", curridx + 1, masterSchdValveDurationMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(500);
								if (irrigationMode[pgmidx] == 2 && (masterSchdValveDurationMins[idx] > 0))
								{
									LCD_Gotoxy(0, 1);
									sprintf(lcd_data, "V%d Dur=%d m PRE", curridx + 1, masterSchdPreWetDurMins[idx]);
									LCD_Puts(lcd_data);
									HAL_Delay(1000);
									LCD_Gotoxy(0, 1);
									sprintf(lcd_data, "V%d Dur=%d m FER", curridx + 1, masterSchdFertgDurMins[idx]);
									LCD_Puts(lcd_data);
									HAL_Delay(1000);
									LCD_Gotoxy(0, 1);
									sprintf(lcd_data, "V%d Dur=%d m FLS", curridx + 1, masterSchdFlushingDurMins[idx]);
									LCD_Puts(lcd_data);
									HAL_Delay(1000);
								}
							}


							if ((sw2 == (SW_START + 4) && (input_timeout < SW_DELAY)))
							{
								// MenuDoneCnt = SW_PGM_SEL_ST+1;

								curridx = curridx + 1; // @@ ##
								idx = idx + 1;

							
								if (curridx == 4)
								{
									week_show_flag = 1;
									sw2 = SW_START + 5;
								}
								else
								{
									sw2 = SW_START + 3;
									goto pb4;
								}	
							} // end if (sw + 4)

						} // end for valve 4
					


						// if exp yes,, if select 5 , for pgB
						if (get_total_v == 5)
						{
						pb5:
							if ((sw2 == SW_START + 3) && (input_timeout < SW_DELAY))
							{
								
								if (displayConfig_flag ==1)
								{
									curridx = 0;
									pgmidx = 1;
									idx = 24;
									displayConfig_flag =0;
								}
			
								
								LCD_Clear();
								LCD_Gotoxy(0, 0);

								if (pgmidx == 1 && irrigationMode[pgmidx] != 2)
									sprintf(lcd_data, "PGM=B         IRRG");
								if (pgmidx == 1 && irrigationMode[pgmidx] == 2)
									sprintf(lcd_data, "PGM=B         FERT");

								LCD_Puts(lcd_data);
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "Valve%d Dur=%d m       ", curridx + 1, masterSchdValveDurationMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(500);
								if (irrigationMode[pgmidx] == 2 && (masterSchdValveDurationMins[idx] > 0))
								{
									LCD_Gotoxy(0, 1);
									sprintf(lcd_data, "V%d Dur=%d m PRE", curridx + 1, masterSchdPreWetDurMins[idx]);
									LCD_Puts(lcd_data);
									HAL_Delay(1000);
									LCD_Gotoxy(0, 1);
									sprintf(lcd_data, "V%d Dur=%d m FER", curridx + 1, masterSchdFertgDurMins[idx]);
									LCD_Puts(lcd_data);
									HAL_Delay(1000);
									LCD_Gotoxy(0, 1);
									sprintf(lcd_data, "V%d Dur=%d m FLS", curridx + 1, masterSchdFlushingDurMins[idx]);
									LCD_Puts(lcd_data);
									HAL_Delay(1000);
								}
							}


							if ((sw2 == (SW_START + 4) && (input_timeout < SW_DELAY)))
							{
								// MenuDoneCnt = SW_PGM_SEL_ST+1;

								curridx = curridx + 1; // @@ ##
								idx = idx + 1;

							
								if (curridx == 5)
								{
									week_show_flag = 1;
									sw2 = SW_START + 5;
								}
								else
								{
									sw2 = SW_START + 3;
									goto pb5;
								}	
							} // end if (sw + 4)

						} // end for valve 5
					

						// if exp yes,, if select 6 , for pgB
						if (get_total_v == 6)
						{
						pb6:
							if ((sw2 == SW_START + 3) && (input_timeout < SW_DELAY))
							{
								
								if (displayConfig_flag ==1)
								{
									curridx = 0;
									pgmidx = 1;
									idx = 24;
									displayConfig_flag =0;
								}
			
								
								LCD_Clear();
								LCD_Gotoxy(0, 0);

								if (pgmidx == 1 && irrigationMode[pgmidx] != 2)
									sprintf(lcd_data, "PGM=B         IRRG");
								if (pgmidx == 1 && irrigationMode[pgmidx] == 2)
									sprintf(lcd_data, "PGM=B         FERT");

								LCD_Puts(lcd_data);
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "Valve%d Dur=%d m       ", curridx + 1, masterSchdValveDurationMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(500);
								if (irrigationMode[pgmidx] == 2 && (masterSchdValveDurationMins[idx] > 0))
								{
									LCD_Gotoxy(0, 1);
									sprintf(lcd_data, "V%d Dur=%d m PRE", curridx + 1, masterSchdPreWetDurMins[idx]);
									LCD_Puts(lcd_data);
									HAL_Delay(1000);
									LCD_Gotoxy(0, 1);
									sprintf(lcd_data, "V%d Dur=%d m FER", curridx + 1, masterSchdFertgDurMins[idx]);
									LCD_Puts(lcd_data);
									HAL_Delay(1000);
									LCD_Gotoxy(0, 1);
									sprintf(lcd_data, "V%d Dur=%d m FLS", curridx + 1, masterSchdFlushingDurMins[idx]);
									LCD_Puts(lcd_data);
									HAL_Delay(1000);
								}
							}


							if ((sw2 == (SW_START + 4) && (input_timeout < SW_DELAY)))
							{
								// MenuDoneCnt = SW_PGM_SEL_ST+1;

								curridx = curridx + 1; // @@ ##
								idx = idx + 1;

							
								if (curridx == 6)
								{
									week_show_flag = 1;
									sw2 = SW_START + 5;
								}
								else
								{
									sw2 = SW_START + 3;
									goto pb6;
								}	
							} // end if (sw + 4)

						} // end for valve 6
					
						// if exp yes,, if select 7 , for pgB
						if (get_total_v == 7)
						{
						pb7:
							if ((sw2 == SW_START + 3) && (input_timeout < SW_DELAY))
							{
								
								if (displayConfig_flag ==1)
								{
									curridx = 0;
									pgmidx = 1;
									idx = 24;
									displayConfig_flag =0;
								}
			
								
								LCD_Clear();
								LCD_Gotoxy(0, 0);

								if (pgmidx == 1 && irrigationMode[pgmidx] != 2)
									sprintf(lcd_data, "PGM=B         IRRG");
								if (pgmidx == 1 && irrigationMode[pgmidx] == 2)
									sprintf(lcd_data, "PGM=B         FERT");

								LCD_Puts(lcd_data);
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "Valve%d Dur=%d m       ", curridx + 1, masterSchdValveDurationMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(500);
								if (irrigationMode[pgmidx] == 2 && (masterSchdValveDurationMins[idx] > 0))
								{
									LCD_Gotoxy(0, 1);
									sprintf(lcd_data, "V%d Dur=%d m PRE", curridx + 1, masterSchdPreWetDurMins[idx]);
									LCD_Puts(lcd_data);
									HAL_Delay(1000);
									LCD_Gotoxy(0, 1);
									sprintf(lcd_data, "V%d Dur=%d m FER", curridx + 1, masterSchdFertgDurMins[idx]);
									LCD_Puts(lcd_data);
									HAL_Delay(1000);
									LCD_Gotoxy(0, 1);
									sprintf(lcd_data, "V%d Dur=%d m FLS", curridx + 1, masterSchdFlushingDurMins[idx]);
									LCD_Puts(lcd_data);
									HAL_Delay(1000);
								}
							}


							if ((sw2 == (SW_START + 4) && (input_timeout < SW_DELAY)))
							{
								// MenuDoneCnt = SW_PGM_SEL_ST+1;

								curridx = curridx + 1; // @@ ##
								idx = idx + 1;

							
								if (curridx == 7)
								{
									week_show_flag = 1;
									sw2 = SW_START + 5;
								}
								else
								{
									sw2 = SW_START + 3;
									goto pb7;
								}	
							} // end if (sw + 4)

						} // end for valve 7
					

						// if exp yes,, if select 8 , for pgB
						if (get_total_v == 8)
						{
						pb8:
							if ((sw2 == SW_START + 3) && (input_timeout < SW_DELAY))
							{
								
								if (displayConfig_flag ==1)
								{
									curridx = 0;
									pgmidx = 1;
									idx = 24;
									displayConfig_flag =0;
								}
			
								
								LCD_Clear();
								LCD_Gotoxy(0, 0);

								if (pgmidx == 1 && irrigationMode[pgmidx] != 2)
									sprintf(lcd_data, "PGM=B         IRRG");
								if (pgmidx == 1 && irrigationMode[pgmidx] == 2)
									sprintf(lcd_data, "PGM=B         FERT");

								LCD_Puts(lcd_data);
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "Valve%d Dur=%d m       ", curridx + 1, masterSchdValveDurationMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(500);
								if (irrigationMode[pgmidx] == 2 && (masterSchdValveDurationMins[idx] > 0))
								{
									LCD_Gotoxy(0, 1);
									sprintf(lcd_data, "V%d Dur=%d m PRE", curridx + 1, masterSchdPreWetDurMins[idx]);
									LCD_Puts(lcd_data);
									HAL_Delay(1000);
									LCD_Gotoxy(0, 1);
									sprintf(lcd_data, "V%d Dur=%d m FER", curridx + 1, masterSchdFertgDurMins[idx]);
									LCD_Puts(lcd_data);
									HAL_Delay(1000);
									LCD_Gotoxy(0, 1);
									sprintf(lcd_data, "V%d Dur=%d m FLS", curridx + 1, masterSchdFlushingDurMins[idx]);
									LCD_Puts(lcd_data);
									HAL_Delay(1000);
								}
							}


							if ((sw2 == (SW_START + 4) && (input_timeout < SW_DELAY)))
							{
								// MenuDoneCnt = SW_PGM_SEL_ST+1;

								curridx = curridx + 1; // @@ ##
								idx = idx + 1;

							
								if (curridx == 8)
								{
									week_show_flag = 1;
									sw2 = SW_START + 5;
								}
								else
								{
									sw2 = SW_START + 3;
									goto pb8;
								}	
							} // end if (sw + 4)

						} // end for valve 8
					


						// if exp yes,, if select 9 , for pgB
						if (get_total_v == 9)
						{
						pb9:
							if ((sw2 == SW_START + 3) && (input_timeout < SW_DELAY))
							{
								
								if (displayConfig_flag ==1)
								{
									curridx = 0;
									pgmidx = 1;
									idx = 24;
									displayConfig_flag =0;
								}
			
								
								LCD_Clear();
								LCD_Gotoxy(0, 0);

								if (pgmidx == 1 && irrigationMode[pgmidx] != 2)
									sprintf(lcd_data, "PGM=B         IRRG");
								if (pgmidx == 1 && irrigationMode[pgmidx] == 2)
									sprintf(lcd_data, "PGM=B         FERT");

								LCD_Puts(lcd_data);
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "Valve%d Dur=%d m       ", curridx + 1, masterSchdValveDurationMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(500);
								if (irrigationMode[pgmidx] == 2 && (masterSchdValveDurationMins[idx] > 0))
								{
									LCD_Gotoxy(0, 1);
									sprintf(lcd_data, "V%d Dur=%d m PRE", curridx + 1, masterSchdPreWetDurMins[idx]);
									LCD_Puts(lcd_data);
									HAL_Delay(1000);
									LCD_Gotoxy(0, 1);
									sprintf(lcd_data, "V%d Dur=%d m FER", curridx + 1, masterSchdFertgDurMins[idx]);
									LCD_Puts(lcd_data);
									HAL_Delay(1000);
									LCD_Gotoxy(0, 1);
									sprintf(lcd_data, "V%d Dur=%d m FLS", curridx + 1, masterSchdFlushingDurMins[idx]);
									LCD_Puts(lcd_data);
									HAL_Delay(1000);
								}
							}


							if ((sw2 == (SW_START + 4) && (input_timeout < SW_DELAY)))
							{
								// MenuDoneCnt = SW_PGM_SEL_ST+1;

								curridx = curridx + 1; // @@ ##
								idx = idx + 1;

							
								if (curridx == 9)
								{
									week_show_flag = 1;
									sw2 = SW_START + 5;
								}
								else
								{
									sw2 = SW_START + 3;
									goto pb9;
								}	
							} // end if (sw + 4)

						} // end for valve 9
					


						// if exp yes,, if select 10 , for pgB
						if (get_total_v == 10)
						{
						pb10:
							if ((sw2 == SW_START + 3) && (input_timeout < SW_DELAY))
							{
								
								if (displayConfig_flag ==1)
								{
									curridx = 0;
									pgmidx = 1;
									idx = 24;
									displayConfig_flag =0;
								}
			
								
								LCD_Clear();
								LCD_Gotoxy(0, 0);

								if (pgmidx == 1 && irrigationMode[pgmidx] != 2)
									sprintf(lcd_data, "PGM=B         IRRG");
								if (pgmidx == 1 && irrigationMode[pgmidx] == 2)
									sprintf(lcd_data, "PGM=B         FERT");

								LCD_Puts(lcd_data);
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "Valve%d Dur=%d m       ", curridx + 1, masterSchdValveDurationMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(500);
								if (irrigationMode[pgmidx] == 2 && (masterSchdValveDurationMins[idx] > 0))
								{
									LCD_Gotoxy(0, 1);
									sprintf(lcd_data, "V%d Dur=%d m PRE", curridx + 1, masterSchdPreWetDurMins[idx]);
									LCD_Puts(lcd_data);
									HAL_Delay(1000);
									LCD_Gotoxy(0, 1);
									sprintf(lcd_data, "V%d Dur=%d m FER", curridx + 1, masterSchdFertgDurMins[idx]);
									LCD_Puts(lcd_data);
									HAL_Delay(1000);
									LCD_Gotoxy(0, 1);
									sprintf(lcd_data, "V%d Dur=%d m FLS", curridx + 1, masterSchdFlushingDurMins[idx]);
									LCD_Puts(lcd_data);
									HAL_Delay(1000);
								}
							}


							if ((sw2 == (SW_START + 4) && (input_timeout < SW_DELAY)))
							{
								// MenuDoneCnt = SW_PGM_SEL_ST+1;

								curridx = curridx + 1; // @@ ##
								idx = idx + 1;

							
								if (curridx == 10)
								{
									week_show_flag = 1;
									sw2 = SW_START + 5;
								}
								else
								{
									sw2 = SW_START + 3;
									goto pb10;
								}	
							} // end if (sw + 4)

						} // end for valve 10
					


						// if exp yes,, if select 11 , for pgB
						if (get_total_v == 11)
						{
						pb11:
							if ((sw2 == SW_START + 3) && (input_timeout < SW_DELAY))
							{
								
								if (displayConfig_flag ==1)
								{
									curridx = 0;
									pgmidx = 1;
									idx = 24;
									displayConfig_flag =0;
								}
			
								
								LCD_Clear();
								LCD_Gotoxy(0, 0);

								if (pgmidx == 1 && irrigationMode[pgmidx] != 2)
									sprintf(lcd_data, "PGM=B         IRRG");
								if (pgmidx == 1 && irrigationMode[pgmidx] == 2)
									sprintf(lcd_data, "PGM=B         FERT");

								LCD_Puts(lcd_data);
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "Valve%d Dur=%d m       ", curridx + 1, masterSchdValveDurationMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(500);
								if (irrigationMode[pgmidx] == 2 && (masterSchdValveDurationMins[idx] > 0))
								{
									LCD_Gotoxy(0, 1);
									sprintf(lcd_data, "V%d Dur=%d m PRE", curridx + 1, masterSchdPreWetDurMins[idx]);
									LCD_Puts(lcd_data);
									HAL_Delay(1000);
									LCD_Gotoxy(0, 1);
									sprintf(lcd_data, "V%d Dur=%d m FER", curridx + 1, masterSchdFertgDurMins[idx]);
									LCD_Puts(lcd_data);
									HAL_Delay(1000);
									LCD_Gotoxy(0, 1);
									sprintf(lcd_data, "V%d Dur=%d m FLS", curridx + 1, masterSchdFlushingDurMins[idx]);
									LCD_Puts(lcd_data);
									HAL_Delay(1000);
								}
							}


							if ((sw2 == (SW_START + 4) && (input_timeout < SW_DELAY)))
							{
								// MenuDoneCnt = SW_PGM_SEL_ST+1;

								curridx = curridx + 1; // @@ ##
								idx = idx + 1;

							
								if (curridx == 11)
								{
									week_show_flag = 1;
									sw2 = SW_START + 5;
								}
								else
								{
									sw2 = SW_START + 3;
									goto pb11;
								}	
							} // end if (sw + 4)

						} // end for valve 11
					

						// if exp yes,, if select 12 , for pgB
						if (get_total_v == 12)
						{
						pb12:
							if ((sw2 == SW_START + 3) && (input_timeout < SW_DELAY))
							{
								
								if (displayConfig_flag ==1)
								{
									curridx = 0;
									pgmidx = 1;
									idx = 24;
									displayConfig_flag =0;
								}
			
								
								LCD_Clear();
								LCD_Gotoxy(0, 0);

								if (pgmidx == 1 && irrigationMode[pgmidx] != 2)
									sprintf(lcd_data, "PGM=B         IRRG");
								if (pgmidx == 1 && irrigationMode[pgmidx] == 2)
									sprintf(lcd_data, "PGM=B         FERT");

								LCD_Puts(lcd_data);
								LCD_Gotoxy(0, 1);
								sprintf(lcd_data, "Valve%d Dur=%d m       ", curridx + 1, masterSchdValveDurationMins[idx]);
								LCD_Puts(lcd_data);
								HAL_Delay(500);
								if (irrigationMode[pgmidx] == 2 && (masterSchdValveDurationMins[idx] > 0))
								{
									LCD_Gotoxy(0, 1);
									sprintf(lcd_data, "V%d Dur=%d m PRE", curridx + 1, masterSchdPreWetDurMins[idx]);
									LCD_Puts(lcd_data);
									HAL_Delay(1000);
									LCD_Gotoxy(0, 1);
									sprintf(lcd_data, "V%d Dur=%d m FER", curridx + 1, masterSchdFertgDurMins[idx]);
									LCD_Puts(lcd_data);
									HAL_Delay(1000);
									LCD_Gotoxy(0, 1);
									sprintf(lcd_data, "V%d Dur=%d m FLS", curridx + 1, masterSchdFlushingDurMins[idx]);
									LCD_Puts(lcd_data);
									HAL_Delay(1000);
								}
							}


							if ((sw2 == (SW_START + 4) && (input_timeout < SW_DELAY)))
							{
								// MenuDoneCnt = SW_PGM_SEL_ST+1;

								curridx = curridx + 1; // @@ ##
								idx = idx + 1;

							
								if (curridx == 12)
								{
									week_show_flag = 1;
									sw2 = SW_START + 5;
								}
								else
								{
									sw2 = SW_START + 3;
									goto pb12;
								}	
							} // end if (sw + 4)

						} // end for valve 12
					



						// for pgB
						if (week_show_flag == 1)
						{

							if ((sw2 == (SW_START + 5) && (input_timeout < SW_DELAY)))
							{

								LCD_Clear();
								LCD_Gotoxy(0, 0);

								LCD_Puts("PGM=B ");

								LCD_Gotoxy(0, 1);
								if ((weekdaysPgmB & 0x01) == 1)
								{
									LCD_Puts("ALL DAYS     ");
								}
								else
								{
									// for(int dayi=1; dayi<8; dayi++)
									{
										LCD_Gotoxy(0, 1);
										if (((weekdaysPgmB >> 1) & 0x01) == 1)
											LCD_Puts("M               ");
										LCD_Gotoxy(2, 1);
										if (((weekdaysPgmB >> 2) & 0x01) == 1)
											LCD_Puts("T               ");
										LCD_Gotoxy(4, 1);
										if (((weekdaysPgmB >> 3) & 0x01) == 1)
											LCD_Puts("W               ");
										LCD_Gotoxy(6, 1);
										if (((weekdaysPgmB >> 4) & 0x01) == 1)
											LCD_Puts("Th               ");
										LCD_Gotoxy(9, 1);
										if (((weekdaysPgmB >> 5) & 0x01) == 1)
											LCD_Puts("F               ");
										LCD_Gotoxy(11, 1);
										if (((weekdaysPgmB >> 6) & 0x01) == 1)
											LCD_Puts("Sa               ");
										LCD_Gotoxy(14, 1);
										if (((weekdaysPgmB >> 7) & 0x01) == 1)
											LCD_Puts("Su               ");
									}
								}
								HAL_Delay(500);
											
								
								if ((sw2 == (SW_START + 6) && (input_timeout < SW_DELAY)))
								{
									
									sw2 = SW_START + 24; // go to exit
									week_show_flag = 0;
									//HAL_Delay(1000);
									break;
								}

							}

						}
				



					
					
					
					} // end pgB disconfig
					else
					{
						LCD_Clear();
						LCD_Puts("PGB");
						LCD_Gotoxy(0, 1);
						LCD_Puts("NOT PROGRAMMED  ");
						HAL_Delay(1000);
						sw2 = SW_START + 24; // go to exit
						break;
					}
				
				}// end pgB nested while



				// displayConfig exit if-block
				if ((sw2 >= SW_START + 23) && (input_timeout < SW_DELAY))
				{
					uint8_t vidx = CurrSchedIdx % 12;
					uint8_t i, u;
					disp = 0;
					sw2 = 0;
					LCD_Clear();
					LCD_Gotoxy(0, 1);
					LCD_Puts("DISPLAY MODE END");
					HAL_Delay(500);
					// LCD_Clear();

					if ((CurrSchedIdx < 12) && (CurrSchedIdx >= 0))
					{
						i = 0;
						u = 1;
					}
					if ((CurrSchedIdx < 24) && (CurrSchedIdx >= 12))
					{
						i = 0;
						u = 2;
					}
					if ((CurrSchedIdx < 36) && (CurrSchedIdx >= 24))
					{
						i = 1;
						u = 1;
					}
					if ((CurrSchedIdx < 48) && (CurrSchedIdx >= 36))
					{
						i = 1;
						u = 2;
					}

					uint8_t vidx1 = CurrSchedIdx % 12;
					if (Run_State)
					{
						if (irrigationMode[CurrSchedIdx / 24] == FERTIGATION_MODE)
						{
							if (pumpstate == 1)
							{
								LCD_Clear();
								LCD_Gotoxy(0, 0);
								sprintf(msg, "PGM-%s ST%d ", pgm_nmnc[i], u);
								LCD_Puts(msg);
								LCD_Gotoxy(0, 1);
								sprintf(msg, "V%d ON PREWET ", vidx1 + 1);
								LCD_Puts(msg);
							}

							uint8_t vidx2 = CurrSchedIdx % 12;
							if (pumpstate == 2)
							{
								LCD_Clear();
								LCD_Gotoxy(0, 0);
								sprintf(msg, "PGM-%s ST%d ", pgm_nmnc[i], u);
								LCD_Puts(msg);
								LCD_Gotoxy(0, 1);
								sprintf(msg, "V%d ON FERTGATION", vidx2 + 1);
								LCD_Puts(msg);
							}

							uint8_t vidx3 = CurrSchedIdx % 12;
							if (pumpstate == 3)
							{
								LCD_Clear();
								LCD_Gotoxy(0, 0);
								sprintf(msg, "PGM-%s ST%d ", pgm_nmnc[i], u);
								LCD_Puts(msg);
								LCD_Gotoxy(0, 1);
								sprintf(msg, "V%d ON FLUSHING ", vidx3 + 1);
								LCD_Puts(msg);
							}
						}
						else
						{
							LCD_Clear();
							LCD_Gotoxy(0, 0);
							sprintf(msg, "PGM-%s ", pgm_nmnc[i]);
							LCD_Puts(msg);
							LCD_Gotoxy(0, 1);
							sprintf(msg, "ST%d V%d ON    %3s ", u, vidx + 1, week[dayIdx - 1]);
							LCD_Puts(msg);
							//				LCD_Clear();
						}
					}
					break;
				}
			}
		}
		else
		{
			LCD_Puts("PGM START TIME  ");
			LCD_Gotoxy(0, 1);
			LCD_Puts("NOT PROGRAMMED  ");
		}
		HAL_Delay(500);
	}
	else
	{
		LCD_Puts("NEED PROGRAM");
	}
	HAL_Delay(500);
}

void cloudSetting()
{

	uint8_t idx;
	uint8_t *Data;
	
	// serverIP read
	Data = (uint8_t *)&server[0];
	for (idx = 0; idx < 16; idx++)
	{
		Data[idx] = EEPROM_ReadByte(EP_server_IP + idx); // Error Flag
	}
	
	// apn read
	Data = (uint8_t *)&apn[0];
	for (idx = 0; idx < 16; idx++)
	{
		Data[idx] = EEPROM_ReadByte(EP_apn + idx); // Error Flag
	}
	
	disp = 1;
	
	LCD_Clear();
	//strcpy(apn, setapn);

	sw2 = 1;
	
	while (disp == 1)
	{
		if ((sw2 == SW_START) && (input_timeout < SW_DELAY))
		{
			
			LCD_Gotoxy(0, 0);
			LCD_Puts("SERVER IP : ");
			LCD_Gotoxy(0, 1);
			sprintf(lcd_data, "%s", server);
			LCD_Puts(lcd_data);
			HAL_Delay(1500);

		}
		
		if ((sw2 == SW_START+1) && (input_timeout < SW_DELAY))
		{
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			LCD_Puts("APN : ");
			LCD_Gotoxy(0, 1);
			sprintf(lcd_data, "%s", apn);
			LCD_Puts(lcd_data);
			HAL_Delay(1500);
			
		}
		
		if ((sw2 == SW_START+2) && (input_timeout < SW_DELAY))
		{
			break;
		}
	
	}
}

#if 0
void pub_msg_sch()
{
	if (hnn[5]==0)
	{
		
		HAL_Delay(2000);
		pub_status_msg();
		
		//===if non-zero then pgA mode set
		if(irrigationMode[0]==1 || irrigationMode[0]==2) 
		{
			HAL_Delay(1000);
			pub_statusA();
		}
		//===if non-zero then pgB mode set
		if(irrigationMode[1]==1 || irrigationMode[1]==2) 
		{
			HAL_Delay(1000);
			pub_statusB();
		}


		hnn[5]=1;
	}
	
	// for every 2 min --- live_msg 
	if ((currRTCtimeMins%4)==0 && live_msg_flg==0)
	{
		pub_live_msg();
		hnn[1]++;
		live_msg_flg=1;
	}
	if ((currRTCtimeMins%4)!=0 && live_msg_flg==1)
	{
		live_msg_flg=0;
	}
	

	
//	curr_v=((CurrSchedIdx % 12) + 1);
//	
//	// after every status change
//	if ((Run_State==1) && ((curr_v <= 12) && (curr_v >= 1)) && (status_chg_flg==0))
//	{
//		
//		
//		//pub_status_msg();
//		//pub_statusA();
//		//pub_statusB();
//	
//		live_msg_flg=1; // for 2min

//			
//		hnn[6]++;
//		status_chg_flg=1;
//		pre_curr_v=curr_v;
//		
//		pub_live_msg();
//		
//		//fert_chg_flg=1; // not run prewet,fert,flush change status
//	}
//	if ((pre_curr_v!=curr_v) && Run_State==1 && status_chg_flg==1)
//	{
//		status_chg_flg=0;
//	}
	
	
//	// in case of fert-mode,, prewet,fert,flush change status
//	if ((Run_State==1) && ((pumpstate <= 3) && (pumpstate >= 1)) && (fert_chg_flg==0))
//	{
//		HAL_Delay(1000);
//		pub_live_msg();
//		
//		pre_pump_state = pumpstate;
//		fert_chg_flg=1;
//	}
//	if ((pre_pump_state!=pumpstate) && (Run_State==1) && (fert_chg_flg==1))
//	{
//		fert_chg_flg=0;
//	}

}

#endif

void EEPROM_LCA_Erase(void)
{
	uint16_t v_eepromAddress_u16;

	for (v_eepromAddress_u16 = EP_MASTER_PUMP_SCHD_STTIME; v_eepromAddress_u16 < 1024; v_eepromAddress_u16++)
	{
		EEPROM_WriteByte(v_eepromAddress_u16, 0xFFu); // Write Each memory location with OxFF
	}
	uint8_t Data = 0;
	EEPROM_WriteNBytes(EP_ERRORFLAG, &Data, 1);			// Error Flag
	EEPROM_WriteNBytes(EP_SCHEDULE_COMPLETE, &Data, 1); // Error Flag
	EEPROM_WriteNBytes(EP_WEEKDAYS_PGMA, &Data, 1);
	EEPROM_WriteNBytes(EP_WEEKDAYS_PGMB, &Data, 1);
	EEPROM_WriteNBytes(EP_VALVECNT, &Data, 1);
	EEPROM_WriteNBytes(EP_CONFIGURED, &Data, 1);
	EEPROM_WriteNBytes(EP_CURRSCHIDX, &Data, 1);
	EEPROM_WriteNBytes(EP_LASTINSTMINS, &Data, 1);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t *Data;
	uint8_t idx, j, i, u;
	uint16_t hrs;
	uint16_t configured = 0xDCBA;
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  
	  //MX_RTC_Init();
	{
		hrtc.Instance = RTC;
		hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
		hrtc.Init.AsynchPrediv = 127;
		hrtc.Init.SynchPrediv = 255;
		hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
		hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
		hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
		if (HAL_RTC_Init(&hrtc) != HAL_OK)
		{
			Error_Handler();
		}
	}
  MX_TIM6_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
//  MX_IWDG_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(RELAY_PWR_GPIO_Port,RELAY_PWR_Pin,1);
	
	// modem_state control pin to set in reset state
//	HAL_GPIO_WritePin(PWR_MODEM_GPIO_Port,PWR_MODEM_Pin,1);
//	
//	testgsm();




	HAL_Delay(25);
	pcf_boot_off(); // ## 
	pcf_inbuilt_boot_off();
	//boot_off(&running); // ## see CLEAR PROGRAM mode 
	
	/* USER CODE BEGIN 2 */

	EEPROM_Init(AT24C16);
	
	
	// EEPROM_Erase();
	/* USER CODE BEGIN 2 */
	LCD_Init();
	lcd_on();
	HAL_Delay(500);
	
	pcf_boot_off(); // ## 
	
	
	HAL_GPIO_WritePin(LCD_LED_GPIO_Port, LCD_LED_Pin, 1);
	// lcd_on();
	LCD_Gotoxy(0, 0);
	LCD_Puts("   AUTOMAT      ");
	LCD_Gotoxy(0, 1);
	LCD_Puts("  INDUSTRIES   ");
	HAL_Delay(1000);
	LCD_Clear();
	LCD_Gotoxy(0, 0);
	LCD_Puts(" AutoDRIP SMART");
	LCD_Gotoxy(0, 1);
	LCD_Puts("  Controller    ");
	
	
	// Start Modem ec 200 

	start_modem();
	HAL_Delay(2000);
	HAL_TIM_Base_Start_IT(&htim14);
	HAL_TIM_Base_Start_IT(&htim6);
	
	LCD_Clear();
	LCD_Puts("STARTING..");
	HAL_Delay(500);
	LCD_Gotoxy(0, 1);
	LCD_Puts("READING CONFIG..");
	
	RTC_TimeShow();
	index1 = 1;
		
	
//		// test gsm process check AT OK
//		LCD_Clear();
//		LCD_Puts("STARTING");
//		HAL_Delay(10);
//		LCD_Gotoxy(0, 1);
//		LCD_Puts("Communication");
//		// TEST GSM MODEM 
//		testgsm();
//		HAL_Delay(500);
//	
//		LCD_Clear();
//		LCD_Puts("DETECTING");
//		HAL_Delay(10);
//		LCD_Gotoxy(0, 1);
//		LCD_Puts("SIM");
//		
//		detect_sim();
//		HAL_Delay(500);
//		
//		LCD_Clear();
//		LCD_Puts("READING");
//		HAL_Delay(100);
//		LCD_Gotoxy(0, 1);
//		LCD_Puts("IMEI");
//		// READ IMEI NO 
//		get_imei();
//		HAL_Delay(500);
//		
//		LCD_Clear();
//		LCD_Puts("SEARCHING ");
//		HAL_Delay(100);
//		LCD_Gotoxy(0, 1);
//		LCD_Puts("NETWORK");
//		// SEARCH NETWORK
//		set_nw_auto();
//		
//		
//		HAL_Delay(2000);
//		// SETTING APN 
//		LCD_Clear();
//		LCD_Puts("SETTING APN");
//		HAL_Delay(100);
//		setAPN(apn);	
//		HAL_Delay(1000);
//		
//		LCD_Clear();
//		LCD_Puts("GETTING ");
//		HAL_Delay(100);
//		LCD_Gotoxy(0, 1);
//		LCD_Puts("NETWORK TIME");
//		get_time_sim();
//		HAL_Delay(1000);
//		
//		 
//		LCD_Clear();
//		LCD_Puts("CONNETING TO");
//		HAL_Delay(100);
//		LCD_Gotoxy(0, 1);
//		LCD_Puts("AUTOMAT SERVER");
//		serverIP(server,port);
//		HAL_Delay(1000);
//		
//		
//		
//		LCD_Clear();
//		LCD_Puts("CONNECTION ");
//		HAL_Delay(100);
//		LCD_Gotoxy(0, 1);
//		LCD_Puts("DONE");
//		mqtt_connect();
//		HAL_Delay(1000);
//		
//		send_boot_msg();
//		pub_msg(topic,send_msg);
//	
//		HAL_Delay(1000);
//		
//		status_msg_pkt();
//		pub_msg(topic,send_msg);
		

		
		
		

		
	printPgmStatus();

	if (pumpRechargingTimeAbsMins > 1440)
	{
		pumpRechargingTimeAbsMins = 1;
	}

	
//	pub_boot_msg();	
//	pub_status_msg();
//	pub_statusA();
//	pub_statusB();
	
	/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */
		
	

		//pub_msg_sch();

//		if (Run_State==1)
//		{
//			HAL_GPIO_WritePin(RELAY_PWR_GPIO_Port,RELAY_PWR_Pin,0);	
//		}
//		else if (Run_State==0)
//		{
//			HAL_GPIO_WritePin(RELAY_PWR_GPIO_Port,RELAY_PWR_Pin,1);
//		}
		
			/* USER CODE BEGIN 3 */
		
		
		if (daynotconfigured == 1)
		{
			LCD_Gotoxy(0, 0);
			LCD_Puts("DAY              ");
			LCD_Gotoxy(0, 1);
			LCD_Puts("NOT PROGRAMMED  ");
			HAL_Delay(1000);
			LCD_Clear();
			printErrStatus();
		}
		if (notconfigured == 1)
		{
			LCD_Gotoxy(0, 0);
			LCD_Puts("PROGRAMMING         ");
			LCD_Gotoxy(0, 1);
			LCD_Puts("NOT DONE  ");
			HAL_Delay(1000);
			LCD_Clear();
			printErrStatus();
		}
		
		// convert to mins => currenttime
		checkflagsPerSec();

		
		// j here decide start time // ##
		if ((CurrSchedIdx < 12) && (CurrSchedIdx >= 0))
		{
			j = 1;
		}
		if ((CurrSchedIdx < 24) && (CurrSchedIdx >= 12))
		{
			j = 2;
		}
		if ((CurrSchedIdx < 36) && (CurrSchedIdx >= 24))
		{
			j = 1;
		}
		if ((CurrSchedIdx < 48) && (CurrSchedIdx >= 36))
		{
			j = 2;
		}

		if (ScheduleComplete == 1)
		{
			if ((Run_State == 0) && (masterSchdValveSttimeMins[CurrSchedIdx] > 0))
			{
				if (currRTCtimeMins == 1439)
				{
					// HAL_NVIC_SystemReset();
					// RTC_TimeShow();
				}
#if 1
				if ((weekdaysPgmA & 0x01) != 1)
				{
					for (i = dayIdx + 1; i < (dayIdx + 8); i++)
					{
						//  int idx = (i%7);
						int idx = i;
						if (i > 7)
						{
							idx = i - 7; // i%7;
						}

						if (((weekdaysPgmA >> idx) & 0x01) == 1)
						{
							NextSchedDayPgmA = i; // idx;
							break;
						}
					}
				}
				else
				{
					NextSchedDay = (dayIdx % 7) + 1;
				}

				if ((weekdaysPgmB & 0x01) != 1)
				{
					for (i = dayIdx + 1; i < (dayIdx + 8); i++)
					{
						// int idx = (i%7);
						int idx = i;
						if (i > 7)
						{
							idx = i - 7; // i%7;
						}
						if (((weekdaysPgmB >> idx) & 0x01) == 1)
						{
							NextSchedDayPgmB = i; // idx;
							break;
						}
					}
				}
				else
				{
					NextSchedDay = (dayIdx % 7) + 1;
				}

				if (((weekdaysPgmA & 0x01) == 1) || ((weekdaysPgmB & 0x01) == 1))
				{
					NextSchedDay = (dayIdx % 7) + 1;
				}
				else
				{

					NextSchedDay = NextSchedDayPgmA;

					if ((NextSchedDayPgmA >= NextSchedDayPgmB) && (NextSchedDayPgmB > 0))
					{
						NextSchedDay = NextSchedDayPgmB;
					}
					else if ((NextSchedDayPgmB > 0) && (NextSchedDayPgmA == 0))
					{
						NextSchedDay = NextSchedDayPgmB;
					}
					else if ((NextSchedDayPgmB > NextSchedDayPgmA) && (NextSchedDayPgmA > 0))
					{
						NextSchedDay = NextSchedDayPgmA;
					}

					if (NextSchedDay > 7)
					{
						NextSchedDay = NextSchedDay - 7;
					}
				}
#endif

				if (CurrSchedIdx < 24)
				{
					// LCD_Clear();
					LCD_Gotoxy(0, 0);
					if (NextSchedDay != 0)
						sprintf(msg, "V%d ST%d PGM-A %3s ", (CurrSchedIdx % 12) + 1, j, week[NextSchedDay - 1]);
					else
						sprintf(msg, "V%d ST%d PGM-A %3s ", (CurrSchedIdx % 12) + 1, j, week[dayIdx - 1]);
					LCD_Puts(msg);
				}
				else
				{
					// LCD_Clear();
					LCD_Gotoxy(0, 0);
					if (NextSchedDay != 0)
						sprintf(msg, "V%d ST%d PGM-B %3s ", (CurrSchedIdx % 12) + 1, j, week[NextSchedDay - 1]);
					else
						sprintf(msg, "V%d ST%d PGM-B %3s ", (CurrSchedIdx % 12) + 1, j, week[dayIdx - 1]);
					LCD_Puts(msg);
				}

				LCD_Gotoxy(0, 1);
				hrs = masterSchdValveSttimeMins[CurrSchedIdx] / 60;
				sprintf(msg, "NEXT SCHED %02d:%02d", masterSchdValveSttimeMins[CurrSchedIdx] / 60, (masterSchdValveSttimeMins[CurrSchedIdx] - hrs * 60));
				LCD_Puts(msg);
				HAL_Delay(2000);
				RTC_TimeShow1();

				printErrStatus();

				//	HAL_Delay(2000);
			}
		}
		else
		{
			// here run_state =0 means pg and valves not runing ##
			if ((Run_State == 0) && (todaySchdValveSttimeMins[CurrSchedIdx] > 0))
			{
				if (effectiveFromToday == 1)
				{
					
					// for pgA --> if ScheduleComplete = 0 , (means, not complete) ##
					if ((weekdaysPgmA & 0x01) != 1)  // means, pgA is not set for all week
					{
						for (i = dayIdx; i < (dayIdx + 8); i++)
						{
							int idx = i;
							if (i > 7)
							{
								idx = i - 7; // i%7;
							}
							// int idx = (i%7);
							//  for (i=(dayIdx);i<8;i++)
							//{
							if (((weekdaysPgmA >> idx) & 0x01) == 1)
							{
								NextSchedDayPgmA = i; // idx;
								break;
							}
						}
					}
					else
					{
						NextSchedDay = dayIdx;
					} // if end e1

					// for pgB --> if ScheduleComplete = 0 , (means, not complete)##
					if ((weekdaysPgmB & 0x01) != 1)
					{
						for (i = dayIdx; i < (dayIdx + 8); i++)
						{
							int idx = i;
							if (i > 7)
							{
								idx = i - 7; // i%7;
							}
							// int idx = (i%7);
							//
							//						for (i=(dayIdx);i<8;i++)
							//						 {
							if (((weekdaysPgmB >> idx) & 0x01) == 1)
							{
								NextSchedDayPgmB = i; // idx;
								break;
							}
						}
					}
					else
					{
						NextSchedDay = dayIdx;
					}  // if end e2


					// NextSchedDay value decider
					if (((weekdaysPgmA & 0x01) == 1) || ((weekdaysPgmB & 0x01) == 1))
					{
						NextSchedDay = dayIdx;
					}
					else
					{

						NextSchedDay = NextSchedDayPgmA;

						if ((NextSchedDayPgmA >= NextSchedDayPgmB) && (NextSchedDayPgmB > 0))
						{
							NextSchedDay = NextSchedDayPgmB;
						}
						else if ((NextSchedDayPgmB > 0) && (NextSchedDayPgmA == 0))
						{
							NextSchedDay = NextSchedDayPgmB;
						}
						else if ((NextSchedDayPgmB > NextSchedDayPgmA) && (NextSchedDayPgmA > 0))
						{
							NextSchedDay = NextSchedDayPgmA;
						}
					}
				} // if effectiveFromToday = 1
				
				// if effectiveFromToday = 0
				else
				{
					if ((weekdaysPgmA & 0x01) != 1)
					{
						for (i = dayIdx; i < (dayIdx + 7); i++)
						{
							int idx = (i % 7) + 1;
							//						 for (i=(dayIdx+1);i<8;i++)
							//						 {
							if (((weekdaysPgmA >> idx) & 0x01) == 1)
							{
								NextSchedDayPgmA = idx;
								break;
							}
						}
					}
					else
					{
						NextSchedDay = (dayIdx % 7) + 1;
					}

					if ((weekdaysPgmB & 0x01) != 1)
					{
						for (i = dayIdx; i < (dayIdx + 7); i++)
						{
							int idx = (i % 7) + 1;
							//						for (i=(dayIdx+1);i<8;i++)
							//						 {
							if (((weekdaysPgmB >> idx) & 0x01) == 1)
							{
								NextSchedDayPgmB = idx;
								break;
							}
						}
					}
					else
					{
						NextSchedDay = (dayIdx % 7) + 1;
					}

					if (((weekdaysPgmA & 0x01) == 1) || ((weekdaysPgmB & 0x01) == 1))
					{
						NextSchedDay = (dayIdx % 7) + 1;
					}
					else
					{

						NextSchedDay = NextSchedDayPgmA;

						if ((NextSchedDayPgmA >= NextSchedDayPgmB) && (NextSchedDayPgmB > 0))
						{
							NextSchedDay = NextSchedDayPgmB;
						}
						else if ((NextSchedDayPgmB > 0) && (NextSchedDayPgmA == 0))
						{
							NextSchedDay = NextSchedDayPgmB;
						}
						else if ((NextSchedDayPgmB > NextSchedDayPgmA) && (NextSchedDayPgmA > 0))
						{
							NextSchedDay = NextSchedDayPgmA;
						}
					}
				} // end if effectiveFromToday 

				if (NextSchedDay > 7)
				{
					NextSchedDay = NextSchedDay - 7;
				}

				// LCD_Clear();
				// if (CurrSchedIdx < 24)
				if (CurrSchedIdx <= 23) // @@ ##
				{
					// LCD_Clear();
					LCD_Gotoxy(0, 0);
					if (NextSchedDay != 0)
						sprintf(msg, "V%d ST%d PGM-A %3s ", (CurrSchedIdx % 12) + 1, j, week[NextSchedDay - 1]);
					else
						sprintf(msg, "V%d ST%d PGM-A %3s ", (CurrSchedIdx % 12) + 1, j, week[dayIdx - 1]);
					LCD_Puts(msg);
				}
				else
				{
					// LCD_Clear();
					LCD_Gotoxy(0, 0);
					if (NextSchedDay != 0)
						sprintf(msg, "V%d ST%d PGM-B %3s ", (CurrSchedIdx % 12) + 1, j, week[NextSchedDay - 1]);
					else
						sprintf(msg, "V%d ST%d PGM-B %3s ", (CurrSchedIdx % 12) + 1, j, week[dayIdx - 1]);
					LCD_Puts(msg);
				}

				//	if ((low_flow_status1 ==0) && (rain_status !=1))
				if (wait_flag == 1)
				{
					LCD_Gotoxy(0, 1);
					hrs = todaySchdValveSttimeMins[CurrSchedIdx] / 60;
					// sprintf(msg,"NEXT SCHED %02d:%02d", todaySchdValveSttimeMins[CurrSchedIdx]/60, (todaySchdValveSttimeMins[CurrSchedIdx]-hrs*60));
					sprintf(msg, "NEXT SCHED %02d:%02d", hrs % 24, (todaySchdValveSttimeMins[CurrSchedIdx] - hrs * 60));
					LCD_Puts(msg);
					HAL_Delay(1500);
				}
				
				// when there is no error and Run_State=0 (means, no valve runing)-- only then show msg [NEXT SCHED]
				if ((Run_State == 0) && (rain_status == 0) && (low_flow_status1 == 0) && (mains_status == 1))
				{
					wait_flag = 1;
				}

				// bal dur display due to error but mains_on ##
				if ((rain_status == 1) || (low_flow_status1 == 1) || ((errorFlag & (1 << LOW_FLOW_2ND_RT)) != 0) || (mains_status == 0))
				{
					//static int balHrs = 0, balMins = 0; // ## ch-ty
					if (Balprint == 1)
					{
						if ((currRTCtimeMins >= todaySchdValveSttimeMins[CurrSchedIdx]) && (currRTCtimeMins < (todaySchdValveSttimeMins[CurrSchedIdx] + todaySchdValveDurationMins[CurrSchedIdx])))
						{
							balMins = todaySchdValveDurationMins[CurrSchedIdx] - (currRTCtimeMins - todaySchdValveSttimeMins[CurrSchedIdx]);							
							balHrs = balMins / 60;
							balMins = (balMins - balHrs * 60);
							if (notconfigured==0) // ## for balMins_msg
							{
								if (daynotconfigured==0 && ScheduleComplete==0)
								{
									balMins_msg = todaySchdValveDurationMins[CurrSchedIdx] - (currRTCtimeMins - todaySchdValveSttimeMins[CurrSchedIdx]);
								}
							}
						}
						else
						{
							balMins = todaySchdValveDurationMins[CurrSchedIdx];
							balHrs = balMins / 60;
							balMins = (balMins - balHrs * 60);
							if (notconfigured==0) // ## for balMins_msg
							{
								if (daynotconfigured==0 && ScheduleComplete==0)
								{
									balMins_msg = todaySchdValveDurationMins[CurrSchedIdx];
								}
							}
						}
						Balprint = 0;
					}
					LCD_Gotoxy(0, 1);

					sprintf(msg, "BAL DUR %02d:%02d   ", balHrs, balMins);
					LCD_Puts(msg);
				}

				//					if(mains_status == 0)
				//					{
				//						static int balHrs=0, balMins=0;
				//						if(Balprint ==1)
				//						{
				//							if ((currRTCtimeMins >= todaySchdValveSttimeMins[CurrSchedIdx]) && (currRTCtimeMins < (todaySchdValveSttimeMins[CurrSchedIdx]+todaySchdValveDurationMins[CurrSchedIdx])))
				//							{
				//								balMins = todaySchdValveDurationMins[CurrSchedIdx] - (currRTCtimeMins - todaySchdValveSttimeMins[CurrSchedIdx]);
				//								balHrs = balMins/60;
				//								balMins =(balMins-balHrs*60);
				//							}
				//							else
				//							{
				//								balMins = todaySchdValveDurationMins[CurrSchedIdx];
				//								balHrs = balMins/60;
				//								balMins =(balMins-balHrs*60);
				//							}
				//							Balprint = 0;
				//						}
				//						LCD_Gotoxy(0,1);
				//
				//						sprintf(msg,"BAL DUR %02d:%02d   ", balHrs, balMins);
				//						LCD_Puts(msg);
				//
				//					}
				HAL_Delay(2000);

				// error display due to second time low flow
				if ((errorFlag & (1 << LOW_FLOW_2ND_RT)) != 0)
				{
					static int lastHrs = 0, lastMins = 0;
					lastHrs = SecondRTTimeInstanceMins / 60;
					lastMins = SecondRTTimeInstanceMins - lastHrs * 60;
					LCD_Clear();
					LCD_Gotoxy(0, 0);
					sprintf(msg, "SYS STOP     %s ", week[SecondRTdayIdx - 1]);
					LCD_Puts(msg);

					LCD_Gotoxy(0, 1);
					LCD_Puts("LOW FLOW 2ND RT ");
					HAL_Delay(2000);
				}

				// HAL_Delay(2000);
				RTC_TimeShow1();
				printErrStatus();

				//	HAL_Delay(2000);
			}
			
			// here run_state =1 means pg and valves runing --> so print the continously print bal_dur during valve runing ##
			else
			{
				//static int balHrs1 = 0, balMins1 = 0; // ## ch-ty
				uint8_t vidx = CurrSchedIdx % 12;
				uint8_t u;

				balMins = todaySchdValveDurationMins[CurrSchedIdx] - (currRTCtimeMins - todaySchdValveSttimeMins[CurrSchedIdx]);
				balHrs = balMins / 60;
				balMins = (balMins - balHrs * 60);
				if (notconfigured==0) // ## for balMins_msg
				{
					if (daynotconfigured==0 && ScheduleComplete==0)
					{
						balMins_msg = todaySchdValveDurationMins[CurrSchedIdx] - (currRTCtimeMins - todaySchdValveSttimeMins[CurrSchedIdx]);
					}
				}

				if ((CurrSchedIdx < 12) && (CurrSchedIdx >= 0))
				{
					u = 1;
				}
				if ((CurrSchedIdx < 24) && (CurrSchedIdx >= 12))
				{
					u = 2;
				}
				if ((CurrSchedIdx < 36) && (CurrSchedIdx >= 24))
				{
					u = 1;
				}
				if ((CurrSchedIdx < 48) && (CurrSchedIdx >= 36))
				{
					u = 2;
				}

				LCD_Gotoxy(0, 1);

				// if notconfigured == 1 then pg not done ##
				if ((notconfigured == 0) && (Run_State == 1))
				{
					if ((runBalflag == 0) && (irrigationMode[CurrSchedIdx / 24] != FERTIGATION_MODE))
					{
						sprintf(msg, "BAL DUR %02d:%02d   ", balHrs, balMins);
						LCD_Puts(msg);
						HAL_Delay(200);
						runBalflag = 1;
					}
					else
					{
						if ((pendIrrgFlag == 0) && (irrigationMode[CurrSchedIdx / 24] != FERTIGATION_MODE))
						{
							sprintf(msg, "ST%d V%d ON    %3s ", u, vidx + 1, week[dayIdx - 1]);
							LCD_Puts(msg);
						}
						runBalflag = 0;    // due to this toggel between if and else block ##
					}
				}
				
				// ## if errors are clear then change wait_flag value from 0 to 1 --> this wait_flag=1 display NEXTsch when there are no error 
				if ((Run_State == 1) && (rain_status == 0) && (low_flow_status1 == 0) && (mains_status == 1))
				{
					wait_flag = 1;
				}
			}
			//				LCD_Gotoxy(7,0);
			//				hrs = currRTCtimeMins/60;
			//				sprintf(msg,"    %02d:%02d ", currRTCtimeMins/60, (currRTCtimeMins-hrs*60));
			//				LCD_Puts(msg);
			//				HAL_Delay(500);
		}  // end if block --> ScheduleComplete (1,0)

		if (pendIrrgFlag == 1)
		{
			if ((CurrSchedIdx < 12) && (CurrSchedIdx >= 0))
			{
				i = 0;
				u = 1;
			}

			if ((CurrSchedIdx < 24) && (CurrSchedIdx >= 12))
			{
				i = 0;
				u = 2;
			}

			if ((CurrSchedIdx < 36) && (CurrSchedIdx >= 24))
			{
				i = 1;
				u = 1;
			}

			if ((CurrSchedIdx < 48) && (CurrSchedIdx >= 36))
			{
				i = 1;
				u = 2;
			}
			LCD_Gotoxy(0, 0);
			sprintf(msg, "V%d %s ST%d", (CurrSchedIdx % 12 + 1), pgm_nmnc[i], u);
			// sprintf(msg,"PGM %s ",pgm_nmnc[i]);
			LCD_Puts(msg);
			LCD_Gotoxy(0, 1);
			sprintf(msg, "PENDG IRRG **%3s", week[prevWeekDay - 1]);
			LCD_Puts(msg);
			LCD_Gotoxy(8, 0);
			hrs = currRTCtimeMins / 60;
			sprintf(msg, "   %02d:%02d", hrs % 24, (currRTCtimeMins - hrs * 60));
			LCD_Puts(msg);
			HAL_Delay(400);
			LCD_Gotoxy(13, 0);
			LCD_Puts(" ");
			HAL_Delay(100);
		}
		else  // if not pending  ##
		{
			if ((ScheduleComplete != 1) && (Run_State == 1))
			{
				LCD_Gotoxy(9, 0);
				hrs = currRTCtimeMins / 60;
				sprintf(msg, "  %02d:%02d ", currRTCtimeMins / 60, (currRTCtimeMins - hrs * 60));
				LCD_Puts(msg);
				HAL_Delay(400);
				LCD_Gotoxy(13, 0);
				LCD_Puts(" ");
				HAL_Delay(100);
			}
		}

		LCD_Gotoxy(16, 0);
		sprintf(msg, "      ", currRTCtimeMins / 60, (currRTCtimeMins - hrs * 60));
		LCD_Puts(msg);
		HAL_Delay(50);

		if ((((weekdays & 0x01) == 1) || (((weekdays >> dayIdx) & 0x01) == 1)) && (ScheduleComplete != 1) && (effectiveFromToday == 1))
		{
			// if((Run_State == RUNNING)  || ((startup_Flag==1) && (minsFlag==1)))
			// if((Run_State == RUNNING)  || (startup_Flag==1) )
			if (((Run_State == RUNNING) || (startup_Flag == 1)) && (halt_alloperation == 0))   // default value Run_state=0, RUNNING=1
			{
				callback_scheduledtimerInterrupt_5mins();
				//minsFlag = 0;  // in ori , no use
			}
		}


		HAL_Delay(100);
		if ((HAL_GPIO_ReadPin(SW_1_GPIO_Port, SW_1_Pin) == 0) && (HAL_GPIO_ReadPin(SW_2_GPIO_Port, SW_2_Pin) == 0))
		{
			sw1 = 0;
			sw2 = 0;
			run_mode = 0;
			mode_flag = 1;
			LCD_Clear();
			LCD_Puts("PLEASE WAIT..           ");
			HAL_Delay(1000);

			while ((mode_flag == 1) && (input_timeout < 100) && (index1 == 1))
			{
				if ((sw1 == SW_START) && (input_timeout < SW_DELAY))
				{
					mode = pgm_mode[sw2];
					sw2_max = 9;
					LCD_Clear();
					LCD_Puts("SELECT MODE:      ");
					LCD_Gotoxy(0, 1);
					sprintf(msg, ": %s                 ", pgm_mode_disp[sw2]);
					LCD_Puts(msg);
					HAL_Delay(1000);
				}

				else if ((sw1 == SW_START + 1) && (mode == 1))
				{
					index1++;
					sw1 = 0;
					//sw2 = 0; // ## in ori, but comment-out --> create issue due to (BCK WITHOUT SAVE)
					run_mode = 0;
					conf_flag = 1;
					LCD_Clear();
					LCD_Puts("START CONFIG MODE  ");
					LCD_Gotoxy(0, 1);
					LCD_Puts("PLEASE WAIT..       ");
					HAL_Delay(1000);
					memset(masterSchdPumpSttimeMins, 0x00, sizeof(masterSchdPumpSttimeMins));
					memset(irrigationMode, 0x00, sizeof(irrigationMode));
					memset(masterSchdValveDurationMins, 0x00, sizeof(masterSchdValveDurationMins));
					parameter_config();
					if (pumpStatus == 0)
					{
						pumpInitiationTimeSecs = 0;
						pumpRechargingTimeAbsMins = 0;
					}
				}
				else if ((sw1 == SW_START + 1) && (mode == 3))
				{
					sw1 = 0;
					sw2 = 0;
					LCD_Clear();
					LCD_Puts("TEST MODE       ");
					LCD_Gotoxy(0, 1);
					LCD_Puts("                ");
					HAL_Delay(1000);
					test_mode();
				}
				else if ((sw1 == SW_START + 1) && (mode == 5))
				{
					sw1 = 0;
					sw2 = 0;
					LCD_Clear();
					LCD_Gotoxy(0, 0);
					LCD_Puts("DELETING DATA");
					LCD_Gotoxy(0, 1);
					//	sprintf(msg,"STATUS: %s            ", choice[sw2]);
					LCD_Puts("PLEASE WAIT...");

					EEPROM_LCA_Erase();

					HAL_Delay(2000);
					//running = 0; // ## in ori
					//valve_output(&running); // ## in ori
					HAL_GPIO_WritePin(RELAY_PWR_GPIO_Port,RELAY_PWR_Pin,1); //off , Run_State=0
					
					LCD_Clear();
					LCD_Gotoxy(0, 0);
					LCD_Puts("PROGRAM   ");
					LCD_Gotoxy(0, 1);
					LCD_Puts("CLEARED...");
					HAL_Delay(200);

					HAL_Delay(2000);
					HAL_NVIC_SystemReset();
				}
				else if ((sw1 == SW_START + 1) && (mode == 4))
				{
					//	sw1=0;
					index1++;
					sw2 = 0;
					LCD_Clear();
					set_date_day_time();
				}

				else if ((sw1 == SW_START + 1) && (mode == 2))
				{
					index1++;
					LCD_Clear();
					pump_setting();
				}

				else if ((sw1 == SW_START + 1) && (mode == 6))
				{
					//		sw1=0;
					sw2 = 0;
					// RTC_TimeShow();
					displayConfig();

					break;
				}
				
				else if ((sw1 == SW_START + 1) && (mode == 7))
				{
					sw2 = 0;
					cloudSetting();
					break;
				}

				else if ((sw1 == SW_START + 1) && (mode == 8))
				{
					// break;
					HAL_NVIC_SystemReset();
				}


				else if ((sw1 == SW_START + 1) && (mode == 9))
				{
					
					uint8_t vidx = CurrSchedIdx % 12;
					uint8_t i, u;
					if ((CurrSchedIdx < 12) && (CurrSchedIdx >= 0))
					{
						i = 0;
						u = 1;
					}
					if ((CurrSchedIdx < 24) && (CurrSchedIdx >= 12))
					{
						i = 0;
						u = 2;
					}
					if ((CurrSchedIdx < 36) && (CurrSchedIdx >= 24))
					{
						i = 1;
						u = 1;
					}
					if ((CurrSchedIdx < 48) && (CurrSchedIdx >= 36))
					{
						i = 1;
						u = 2;
					}

					uint8_t vidx1 = CurrSchedIdx % 12;
					if (Run_State)
					{
						if (irrigationMode[CurrSchedIdx / 24] == FERTIGATION_MODE)
						{
							if (pumpstate == 1)
							{
								LCD_Clear();
								LCD_Gotoxy(0, 0);
								sprintf(msg, "PGM-%s ST%d ", pgm_nmnc[i], u);
								LCD_Puts(msg);
								LCD_Gotoxy(0, 1);
								sprintf(msg, "V%d ON PREWET ", vidx1 + 1);
								LCD_Puts(msg);
							}

							uint8_t vidx2 = CurrSchedIdx % 12;
							if (pumpstate == 2)
							{
								LCD_Clear();
								LCD_Gotoxy(0, 0);
								sprintf(msg, "PGM-%s ST%d ", pgm_nmnc[i], u);
								LCD_Puts(msg);
								LCD_Gotoxy(0, 1);
								sprintf(msg, "V%d ON FERTGATION", vidx2 + 1);
								LCD_Puts(msg);
							}

							uint8_t vidx3 = CurrSchedIdx % 12;
							if (pumpstate == 3)
							{
								LCD_Clear();
								LCD_Gotoxy(0, 0);
								sprintf(msg, "PGM-%s ST%d ", pgm_nmnc[i], u);
								LCD_Puts(msg);
								LCD_Gotoxy(0, 1);
								sprintf(msg, "V%d ON FLUSHING ", vidx3 + 1);
								LCD_Puts(msg);
							}
						}
						else
						{
							LCD_Clear();
							LCD_Gotoxy(0, 0);
							sprintf(msg, "PGM-%s ", pgm_nmnc[i]);
							LCD_Puts(msg);
							LCD_Gotoxy(0, 1);
							sprintf(msg, "ST%d V%d ON    %3s ", u, vidx + 1, week[dayIdx - 1]);
							LCD_Puts(msg);
							//				LCD_Clear();
						}
					}
					//HAL_NVIC_SystemReset(); // ## @@
					break;
				}

				else
				{
					sw1 = SW_START;
				}
			} // while end
		}
		//		LCD_Gotoxy(0,0);
		//	  LCD_Puts("Executing.....  ");
	}
	
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_HIGH);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x70418afe;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = get_hrs;
  sTime.Minutes = get_mins;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = get_day;
  sDate.Month = get_month;
  sDate.Date = get_date;
  sDate.Year = get_year;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 32000-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 5000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 32000-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 60000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GSM_DTR_Pin|PUMP_R_Pin|RELAY_PWR_Pin|LCD_D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EXP_OUT_Pin|LCD_D6_Pin|LCD_D5_Pin|LCD_D4_Pin|LCD_RS_Pin|LCD_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_LED_GPIO_Port, LCD_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GSM_RI_Pin MAINS_DT_Pin LOW_FLOW_Pin RAIN_SENSOR_Pin */
  GPIO_InitStruct.Pin = GSM_RI_Pin|MAINS_SN_Pin|LOW_FLOW_Pin|RAIN_SENSOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GSM_DTR_Pin PUMP_R_Pin RELAY_PWR_Pin LCD_D7_Pin */
  GPIO_InitStruct.Pin = GSM_DTR_Pin|PUMP_R_Pin|LCD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	
	GPIO_InitStruct.Pin = RELAY_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_5_Pin SW_4_Pin SW_3_Pin */
  GPIO_InitStruct.Pin = SW_5_Pin|SW_4_Pin|SW_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_2_Pin SW_1_Pin */
  GPIO_InitStruct.Pin = SW_2_Pin|SW_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : EXP_OUT_Pin LCD_D6_Pin LCD_D5_Pin LCD_D4_Pin
                           LCD_LEDB8_Pin */
  GPIO_InitStruct.Pin = EXP_OUT_Pin|LCD_D6_Pin|LCD_D5_Pin|LCD_D4_Pin|LCD_EN_Pin|LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : FLOW_1_Pin */
  GPIO_InitStruct.Pin = FLOW_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FLOW_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_LED_Pin */
  GPIO_InitStruct.Pin = LCD_LED_Pin|PWR_MODEM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */


void valve_output(uint8_t *data, uint8_t *pcf_data)
{

	shift_data = *data;
	pcf_dataa = *pcf_data;
	

	//pcf_return_inbuilt = pcf_inbuilt_on(W, shift_data, 0);
	
	if (shift_data==0xD0) //irrg v1
	{
		pcf_return_inbuilt = pcf_inbuilt_on(0x68);
	}
	else if (shift_data==0xC8) //irrg v2
	{
		pcf_return_inbuilt = pcf_inbuilt_on(0x64);
	}
	else if (shift_data==0xC4) //irrg v3
	{
		pcf_return_inbuilt = pcf_inbuilt_on(0x62);
	}
	else if (shift_data==0xC2) //irrg v4
	{
		pcf_return_inbuilt = pcf_inbuilt_on(0x61);
	}
	else if (shift_data==0x00)
	{
		pcf_return_inbuilt = pcf_inbuilt_on(0x00);
	}
	else if (shift_data==0xC0) // exp irr prewet, flush
	{
		pcf_return_inbuilt = pcf_inbuilt_on(0x60);
	}
	else if (shift_data==0xA0) // exp fert
	{
		pcf_return_inbuilt = pcf_inbuilt_on(0x50);
	}
	else if (shift_data==0xB0)  // fert v1
	{
		pcf_return_inbuilt = pcf_inbuilt_on(0x58);
	}
	else if (shift_data==0xA8)  // fert v2
	{
		pcf_return_inbuilt = pcf_inbuilt_on(0x54);
	}
	else if (shift_data==0xA4)  // fert v3
	{
		pcf_return_inbuilt = pcf_inbuilt_on(0x52);
	}
	else if (shift_data==0xA2)  // fert v2
	{
		pcf_return_inbuilt = pcf_inbuilt_on(0x51);
	}

	
	// ## if not required to correct the order of pcf extended board
	//pcf_return = pcf_on(W, pcf_dataa, 0);
	
	// ## to correct the order of pcf extended board
	if (pcf_dataa==PCF_5)
	{
		pcf_return = pcf_on(0x02);
		pcf_on(0x00);
	}
	else if (pcf_dataa==PCF_6)
	{
		pcf_return = pcf_on(0x04);
	}
	else if (pcf_dataa==PCF_7)
	{
		pcf_return = pcf_on(0x08);
	}
	else if (pcf_dataa==PCF_8)
	{
		pcf_return = pcf_on(0x01);
	}
	else if (pcf_dataa==PCF_9)
	{
		pcf_return = pcf_on(0x10);
	}
	else if (pcf_dataa==PCF_10)
	{
		pcf_return = pcf_on(0x80);
	}
	else if (pcf_dataa==PCF_11)
	{
		pcf_return = pcf_on(0x40);
	}
	else if (pcf_dataa==PCF_12)
	{
		pcf_return = pcf_on(0x20);
	}
	else if (pcf_dataa==0x00)
	{
		pcf_return = pcf_on(0x00);
	}
	else
	{
		pcf_return = pcf_on(0x00);
	}
	
	HAL_Delay(10); 

}







void stop_irrigation(int stop_err)
{
	// write reset
	// irrifation stop flag to 1
}


int pump_initialization()
{
	// pump_initiation=1;
	uint8_t tempFlag = 0XEF; //!(1<<PUMP_RECHARGING_TIME_ACTIVE);
	int status = 0;

	errorFlag &= tempFlag;
	// start pump
	//  wait for pump initiation
	LCD_Clear();
	LCD_Gotoxy(0, 0);
	LCD_Puts("PUMP INIT>>> ");

	pumpinitCnt++;

	if (pumpInitiationTimeSecs > 90)
	{
		pumpInitiationTimeSecs = 5;
	}

	HAL_Delay(pumpInitiationTimeSecs * 1000 + 1);
	pump_initiation = 0;
	HAL_Delay(500);

	low_flow_status1 = HAL_GPIO_ReadPin(LOW_FLOW_GPIO_Port, LOW_FLOW_Pin);

	//			 if (low_flow_status1 == 1)
	//			 {
	//					halt_alloperation =0;
	//			 }

	// low_flow_status = !low_flow_status1;
	low_flow_status = low_flow_status1;

	if (low_flow_status == 1)
	{
		LCD_Gotoxy(0, 1);
		LCD_Puts("Retry....");

		// Go for recharging time
	}
	
	return status;
}


void lcd_on()
{
//	HAL_GPIO_WritePin(LCD_VCC_GPIO_Port, LCD_VCC_Pin, 1);
//	HAL_Delay(1);
	//HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	//htim15.Instance->CCR1 = 40000;
//	HAL_Delay(200);
	HAL_GPIO_WritePin(LCD_LED_GPIO_Port, LCD_LED_Pin, 1);
//	LCD_Init();
//	HAL_Delay(2);
	lcd_on_flag = 1;
	lcd_timeout = 0;
}

void lcd_off()
{
	HAL_GPIO_WritePin(LCD_LED_GPIO_Port, LCD_LED_Pin, 0);

	//	HAL_GPIO_WritePin(LCD_VCC_GPIO_Port,LCD_VCC_Pin,0);
	//HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
	//htim15.Instance->CCR1 = 0;
//	HAL_GPIO_WritePin(LCD_VCC_GPIO_Port, LCD_VCC_Pin, 0);
	lcd_on_flag = 0;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
