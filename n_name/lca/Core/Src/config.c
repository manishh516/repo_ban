/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : config.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "config.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_16x2.h"
#include "eeprom_AT24xxx.h"
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

/* USER CODE BEGIN PV */
// EEPROM Address

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */
extern IWDG_HandleTypeDef hiwdg;

uint8_t mode = 0;
uint16_t currRTCtimeMins = 0;
uint8_t dayIdx = 1;
uint8_t mains_status=0;
uint8_t prevWeekDay = 0;

uint8_t weekday[2][8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t weekdays = 0;

// 0 0 0 0 0 0 0 0	1		(all days)
// x x x x x x x x  0   (select days)

//uint8_t pgm_mode[13] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13}; // ch-type
uint8_t pgm_mode[8] = {1, 2, 3, 4, 5, 6, 7, 8}; // ch-type

//char pgm_mode_disp[14][16] = {"PROGRAMMING","BLUETOOTH","PUMP SETTING", "TEST", "DATE AND TIME", "CLEAR PROGRAM", "DISPLAY PGM", "CLOUD STATUS", "CLOUD SETTING", "DIAGNOSTIC NW", "SENSOR CONFIG", "RESTART", "EXIT"};
//char pgm_mode_disp[8][16] = {"TEST", "DATE AND TIME", "CLOUD STATUS", "CLOUD SETTING", "DIAGNOSTIC NW", "RESTART", "EXIT"};
char pgm_mode_disp[9][16] = {"TEST", "DATE AND TIME", "CLOUD STATUS", "CLOUD SETTING", "DIAGNOSTIC NW", "SENSOR CONFIG", "RESTART", "EXIT"};

char choice[2][4] = {"YES", "NO "};

char week[7][10] = {"MONDAY", "TUESDAY", "WEDNESDAY", "THURSDAY", "FRIDAY", "SATURDAY", "SUNDAY"};

char months[12][10] = {"JANUARY", "FEBRUARY", "MARCH", "APRIL", "MAY", "JUNE", "JULY", "AUGUST", "SEPTEMBER", "OCTOBER", "NOVEMBER", "DECEMBER"};
//char months[12][4] = {"JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"};
uint16_t set_DurHrs[24] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23};
uint16_t set_DurMins[15] = {00, 01, 02, 03, 05, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55}; // ori
uint16_t set_DurMins_recharge[15] = {01, 02, 03, 04, 05, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55}; // ##
uint8_t set_weekdays[2] = {1, 0};


//int set_DurSecs[] = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90}; //ori
int set_DurSecs[] = {5, 8, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90}; // ##
	
char lcd_data[100] = {'\0'}; // cc -flush_flag-interval_tick-dip_duration-loop_flag-manual_sw_status-dp_only-dp_sw_status,, not-sure--run_mode,, 
int duration_tick; 
uint8_t run_mode, conf_flag; // ch-type

char msg[40] = {'\0'};
volatile uint16_t sw1, sw2, sw3, swbk;
uint16_t swtemp;
int durHrs, durMins, cnfgDn = 0, pgmcnfgDn = 0;
int MenuDoneCnt = 0, backflag = 0, mains_off_flag = 0;
uint8_t mode_flag = 1; // ch-type
uint8_t index1 = 0;

uint8_t date[31] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31};
uint8_t month[12] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
uint8_t year[7] = {25, 26, 27, 28, 29, 30, 31};
uint8_t mins[60] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59};
uint8_t day[7] = {1, 2, 3, 4, 5, 6, 7};
uint8_t d_t = 1; //## ch-type

int get_date, get_month, get_year, get_hrs, get_mins, get_day;




extern char server[]; // ##
extern char apn[]; // ##
extern uint8_t lcd_prob;
extern uint8_t Run_State;














 




uint16_t input_timeout, duration;
uint8_t menu, sw1_max = 64, sw2_max = 12, swbk_max = 12;
uint8_t lcd_retain_flag1 = 0; // ch-type
uint8_t lcd_retain_flag2 = 0; // ch-type

extern void lcd_on(void);
extern int lcd_on_flag; // not ch-ty
int lcd_timeout = 0; // not ch-ty

// water meter new_rr7
uint8_t TotalFlow1Counter;
extern uint8_t pulse_count1,seconds_count;
extern uint32_t Total_Flow1_count;
extern uint8_t water_flg;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*@brief -> GPIO_Pin switchs(SW_1_Pin,SW_2_Pin,SW_3_Pin,SW_4_Pin)
 */
extern uint8_t sw5_ip_flg;

uint16_t SW_UPDATE_INTERVAL=600;  // Minimum 700 ms between updates
uint32_t last_update_time_sw1 = 0;
uint32_t current_time=0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	////uint32_t current_time = HAL_GetTick();
	
	if(GPIO_Pin==SW_1_Pin)
	{
		if(lcd_on_flag==0)
		{
			lcd_prob=1;
			lcd_on();
		}
		
		current_time = HAL_GetTick();
		
		if (((current_time - last_update_time_sw1) > SW_UPDATE_INTERVAL))
		{
			last_update_time_sw1 = current_time;
			
			lcd_timeout=0;
			swtemp=sw2;
			sw1++;
			if(sw1>=sw1_max)
			{
				sw1=0;
			}
			input_timeout=0;
			sw2=0;
		}
	}
	
	//Scroll Up
	if(GPIO_Pin==SW_2_Pin)
	{
		if(lcd_on_flag==0)
		{
			lcd_prob=1;
			lcd_on();
		}
		
		lcd_timeout=0;
		sw3++;
		sw2++;
		if(sw2>=sw2_max)
		{
			sw2=0; 
		}
		input_timeout=0;
		swtemp=0;
		swbk=0;
	}
	

	//Scroll Down
	if(GPIO_Pin==SW_3_Pin)
	{
		if(lcd_on_flag==0)
		{
			lcd_prob=1;
			lcd_on();
		}
	lcd_timeout=0;
		sw3--;
		swbk++;
		if(swbk>=swbk_max)
		{
			swbk=0; 
		}
		if (sw2 != 0)
		{
			sw2--;
		}		
		else
		{
		   sw2= sw2_max-1;
		}
	}	
	
	
	//Menu Back
	if(GPIO_Pin==SW_4_Pin)
	{
		if(lcd_on_flag==0)
		{
			lcd_prob=1;
			lcd_on();
		}
		lcd_timeout=0;
	  swbk++;
		if(swbk>=swbk_max)
		{
			swbk=0; 
		}
		if (sw1 != 0)
		{
			sw1--;
		}		

		input_timeout=0;
		cnfgDn=0;
		backflag=1; 
		sw2=0;
		swtemp=0;
	}
	
	if(GPIO_Pin==SW_5_Pin)
	{
		if (sw5_ip_flg==1)
		{	
			sw2 = sw2+10;
		}
	}
	
	//==== water meter
	//if(GPIO_Pin == FLOW_1_Pin){   // water meter  // ori
	if((GPIO_Pin == FLOW_1_Pin) && (Run_State==1)){ 
		TotalFlow1Counter++;
		if(TotalFlow1Counter==10)
		{	
			Total_Flow1_count++;    // for volume of data store in eeprom
			TotalFlow1Counter=0;
			water_flg=1;
		}
		pulse_count1++;
	}
	
}




/* USER CODE END 0 */

#ifdef USE_FULL_ASSERT
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
tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

