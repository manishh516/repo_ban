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

//#include "PCF8574.h" // ## @
//#include "stdbool.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "ec_200_lib.h"
#include "pkt_pub.h"
#include "remote_config.h"
#include "mcp23017.h"

char buff[2];  // for test
char buff1[2];
uint32_t Head = 0;
uint32_t Tail = 0;
char buff_time[43];
char buff_time1[6]="00000";
char buff_date[11]="0000000000";
uint8_t Sim_detected_flag = 0;
//uint8_t SIM_disable_bit=0; // ##
uint16_t CNDTR_Buff_check = 0;
extern char j_str[700];
uint16_t CNDTR_Buff_check_previous = 0;
char check_sub_data[150] = {0};
char stat_err_buff[150] = {0}; // ## 
char low_2nd_err_buff[150]= {0}; // ##

uint8_t stat_error_chk_flag=0; // ## 
uint8_t low_2nd_error_chk_flag=0; // ##

uint8_t exemp_case_flag=0;
uint8_t sub_receive_flag = 0;

uint8_t minute_counter_mqtt_check = 0;


char buff_Check_conn[50] = {0};
uint8_t mqtt_connected_flag = 0;
uint8_t minute_counter_mqtt_check_flag = 0;

uint8_t signal_strength_flag = 0;
char buff_signal_strength[12] = "0000000000/";
char buff_signal_strength_data[2] ={0};

int duration_tick_flag=0;
int duration_tick_flag_B=0;
//int dur_tick_flag_time_sync=0;

void mqtt_valve_output();
uint8_t get_val_M=0;
uint8_t val_M=0;
uint8_t val_PB[6]={0,0,0,0,0,0};
uint8_t val_PA[8]={0,0,0,0,0,0,0,0};
//uint8_t vv_2,vv_3,vv_4,vv_5,vv_6,vv_7,vv_8,vv_9,vv_10,vv_11,vv_12,vv_13,vv_14,vv_15;
uint8_t vv_[16] = {0};
char extracted_buffer5[500] = {0}; //new
extern uint8_t mode;
extern uint8_t pgm_mode[8];
extern char pgm_mode_disp[9][16];
extern uint8_t dayIdx;
extern uint8_t prevWeekDay;
uint8_t halt_alloperation=0;
uint8_t no_cloud_val_off_flg=0;

extern uint8_t cntr_reset_flg;

uint8_t menu_flg=0;

uint8_t pumpInitiationTimeSecs = 5; //=90;
uint8_t pit_val_chk=0;
uint8_t pit_count=0;
uint8_t pit_count_up_flg=0;
uint8_t lf_val_stop_flg=0;

// uart
uint8_t timer_uart_chk_flg=0; // 
uint8_t uart_reset_timer=0; // 
uint8_t uart_reset_flg=0; // 
uint8_t min_count=0;

// val status
char val_SS_M[2]={0};
char val_SS_PB[6]={0};
char val_SS_PA[8]={0};

// pkt time
uint16_t custom_pkt_time=3;
uint16_t custom_pkt_time_val_chk=0;
uint8_t pkt_t_store_bit=0;

int day_reset = 0, reset_counter = 0;

// sim 
uint8_t no_sim_counter=0;

//==== // ^^test
//uint16_t qmt_recv_count=0;
//uint16_t mcp_trigger_count=0;

uint8_t live_pkt_reconnect_flag=0;
//uint8_t stat_er_count=0;

// sensors ##
uint8_t sensor_status=0;
uint8_t Run_State=0;
uint8_t rain_read_disable_flg=0;
uint8_t rain_status1=0;
uint8_t rain_status=0;

uint8_t lf_read_disable_flg=0;
uint8_t low_flow_status1=0;
uint8_t low_flow_status=0;
uint8_t low_flow_off_flg=0;
//uint8_t lf_trigger_once=0;




uint32_t Head_data6 = 0;
uint32_t Tail_data6 = 0;
uint32_t Head_data5 = 0;
uint32_t Tail_data5 = 0;
uint32_t Head_data4 = 0;
uint32_t Tail_data4 = 0;
uint32_t Head_data3 = 0;
uint32_t Tail_data3 = 0;
uint32_t Head_data2 = 0;
uint32_t Tail_data2 = 0;
uint32_t Head_data1 = 0;
uint32_t Tail_data1 = 0;
uint32_t Head_data = 0;
uint32_t Tail_data = 0;

uint8_t modem_restart_flag=0;


 
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


/* USER CODE BEGIN PFP */



// extern int  weekdays[8]; // 1 0 0 0 0 0 0 0 0  (all days)
//  0 x x x x x x x x   (select days)
extern uint8_t weekdays;

extern char choice[2][4];
extern char week[7][10];
extern char months[12][10];

extern uint16_t set_DurHrs[24];
extern uint16_t set_DurMins[12];
extern int set_weekdays[2];


extern int set_DurSecs[];
extern char lcd_data[100]; // cc -flush_flag-interval_tick-dip_duration-loop_flag-manual_sw_status-dp_only-dp_sw_status,, not-sure--run_mode,, 


extern int duration_tick; 
extern uint8_t run_mode, conf_flag; // ch-type
extern char msg[40];
extern volatile uint16_t sw1, sw2, sw3, swbk;
extern uint16_t swtemp;
extern int durHrs, durMins, cnfgDn, pgmcnfgDn;
extern int MenuDoneCnt, backflag, mains_off_flag;
extern uint8_t mode_flag;

extern uint8_t date[31];
extern uint8_t month[12];

extern uint8_t year[7];
extern uint8_t mins[60];
extern uint8_t day[7];
extern uint8_t d_t;
extern uint8_t index1;

extern int get_date, get_month, get_year, get_hrs, get_mins, get_day;

int k_t =0,m_t = 0,F_data = 0,C_data =20,D_data =0;


uint8_t vidxi=0;

extern uint16_t currRTCtimeMins;


extern uint8_t mains_off_Signal_Flag;
extern uint8_t SpecialEv_ON_Flag;
extern uint8_t SpecialEv_OFF_Flag;


extern void addtime(int *SumH, int *SumM, int *BH, int *BM);


void RTC_TimeShow(void);
void RTC_TimeShow1(void);
void RTC_TimeShow_get(void);


uint8_t uart_ERROR;

void uct_EEPROM_READ(void);
void uct_EEPROM_FIXWRITE(int);
void uct_EEPROM_UPDATEDWRITE(void);

void lcd_on(void);
void lcd_off(void);

void APN_SET_REMOTE(char *apn_str_value);

void nw_err_validity_boot_fn (char *key_str); // for boot
void network_boot_chk(); // for boot
void nw_err_validity_fn (char *key_str);
void network_chk();
//void //&&**live_values();
void start_modem_config();

void SET_PASS_CODE();
void j_str_validity_fn (char *key_str);

extern uint8_t mains_status;
extern uint8_t mains_status1;
extern RTC_TimeTypeDef sTime, gTime;


extern uint16_t input_timeout, duration;
extern uint8_t menu, sw1_max, sw2_max, swbk_max;
extern uint8_t lcd_retain_flag1;
extern uint8_t lcd_retain_flag2;


int lcd_on_flag = 0;
extern int lcd_timeout;

char apn[65]= "airtelgprs.com"; // 65<--due to iot apn set
//char server[16]="15.206.24.163";
char server[16]="103.93.94.103";
char port[6]="1883";

extern char send_msg[220];


uint8_t rep_v=0;	  // ##
uint8_t rep_stop=0; // ##
uint8_t reconnect_cn=0; //counter mqtt reconnect 
uint8_t retry_flg=0;
uint8_t sim_problem=0;

uint8_t time_ok=0;


// for ipSet()
char c_ip[16]={0}; // ## cloud ip
uint8_t ip_save_flg =0;
uint8_t ip_no_yes[2] = {1, 2};
uint8_t ip_setup=0;
uint8_t ip_setup_w_loop=0;
uint8_t sw5_ip_flg=0; // to enable/disale (0/1) --> the 5th button 
uint8_t oct_1= 0;
uint8_t oct_2= 0;
uint8_t oct_3= 0;
uint8_t oct_4= 0;
char choice_ip[2][4] = {"NO", "YES "};

// lcd char prob
uint8_t lcd_prob=0;
uint8_t LCD_Init_timer_flg=0;

//j_str validity vars
uint8_t key_idx=0; // ##
uint8_t j_str_validity_flag=0; //##
uint8_t j_str_validity_flag_2=0; //##

//cloud_set_menu() varss
uint8_t m_id=0;
uint8_t factory_reset_flg=0;

uint8_t dig_id=0;

uint8_t low_flow_corr_dur_flg=0;

// alert ??
//uint8_t lf=0;
uint8_t lff_status=0;
uint8_t lf_alert_state_flg=0;
uint8_t rain_alert_flg=0;
uint8_t reg_state_2_run=0;
uint8_t reg_state_2_run_pkt_flg=0;
uint8_t lf_corr_flg=0;
//uint8_t dur_reconn_flg=0;

uint16_t k_cen=0;
uint16_t j_cen=0;

//mcp vars
uint8_t mcp_return=0;
uint8_t mcp_mode_return=0;
uint8_t mcp_error_flg=0;
//uint8_t mcp_read;
uint8_t t_var=0;
uint8_t dataa=0x01;
uint8_t pgA_complete_flg=0;
uint8_t shift_data=0; //## fn valve_output()
uint8_t pcf_dataa=0; // ## fn valve_output()
uint8_t mcp_pre_val_A=0; // ##
uint8_t mcp_pre_val_B=0; // ##


uint8_t after_flush_valve=0;

uint8_t Time_at_sim_disable_flg=0;

uint8_t pending_WR_flg=0;



// pkt vars
extern uint8_t remote_flg; // ## remote flg 1=remote_config enable, 0=disable
extern uint16_t ex_var[24];
uint8_t remote_pg_store=0;
uint8_t remote_pg_store_BT=0;
uint16_t pkt_min_count=0; //pkt timer

uint8_t modem_reboot_count_on_mains=0;

uint8_t IP_store_bit=0;
uint8_t APN_store_bit=0;
uint8_t PORT_store_bit=0;

//// pub_msg_sch ##
uint8_t status_msg_flag=0;
uint8_t status_chg_flg=0;
uint8_t pre_pumpstate=0;
uint8_t live_msg_flg=0;
uint8_t vs=0; // for live_msg



// PASS_CODE vars
uint8_t pass_flg=0;
uint8_t sys_fn_flg=0; // to enable/disable 1/0 , the cloud setting menu --> after passcode input  
uint8_t pass_bit_store=0;
uint8_t pass_ev[4]={1,2,3,4};
uint16_t entered_pass = 0; // Combine array elements into a single number

// emergency vars
uint8_t emergency_bt_flg=0;
uint8_t emergency_val_off_flg=0;
//uint8_t emergency_flg_disp_flg=0;
uint8_t emergency_blck_flg=0; // if block flg, on that it run only once

uint8_t dma_enable=0;


// net_chk vars
char nw_err_buff[60]={0}; // 
uint8_t nw_err_validity_boot_flag=0;
uint8_t nw_err_validity_flag=0;

// water metter vars
uint8_t pulse_count1=0,seconds_count=0,flow_rate1=0;
uint32_t Total_Flow1_count=0;
uint32_t PRE_Total_Flow1_count=0;
uint8_t water_flg=0;
uint8_t flow_pulse_store_flg=0; // moist
extern uint8_t TotalFlow1Counter;

//Network Date and time
char buff_date_yy[5]={NULL};
char buff_date_dd[3]={NULL};
char buff_date_mm[3]={NULL};
char buff_time1_hrs[3]={NULL};
char buff_time1_min[3]={NULL};
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int pump_initialization();
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// 5sec timer for IO status
	if (htim->Instance == TIM6)
	{
		// press both sw4 and sw5 for reset the device but --> after 2nd low flow error --> this not work ## 
		if ((HAL_GPIO_ReadPin(SW_4_GPIO_Port, SW_4_Pin) == 0) && (HAL_GPIO_ReadPin(SW_5_GPIO_Port, SW_5_Pin) == 0))
		{
			HAL_NVIC_SystemReset();
		}		
		

		duration_tick++;
		lcd_timeout++;
		
		//if((pumpInitiationTimeSecs/5)>pit_count && Run_State==1)
		if((pumpInitiationTimeSecs/5)>pit_count && val_M==1)
		{
			pit_count++;
			
		}
		//else if (pumpInitiationTimeSecs<=pit_count && Run_State==1)
		else if (val_M==1)
		{
			pit_count_up_flg=1;
		}

		// ori
		if (lcd_timeout > LCD_MAX_TIMEOUT && mains_status == 0 && lcd_on_flag == 1)
		{
			lcd_off();
		}

		mains_status = HAL_GPIO_ReadPin(MAINS_SN_GPIO_Port, MAINS_SN_Pin);
		
		
		if (rain_read_disable_flg!=1)
		{

				rain_status1 = HAL_GPIO_ReadPin(RAIN_SENSOR_GPIO_Port, RAIN_SENSOR_Pin);
				rain_status = !rain_status1;
		}
		
		
		//if (lf_read_disable_flg!=1 && Run_State==1 && pit_count_up_flg==1)
		if (lf_read_disable_flg!=1 && val_M==1 && pit_count_up_flg==1)
		{

			low_flow_status1 = HAL_GPIO_ReadPin(LOW_FLOW_GPIO_Port, LOW_FLOW_Pin);
			low_flow_status = low_flow_status1;
			
			if (low_flow_status==1)
			{
				// val stop fn run flg
				low_flow_off_flg=1;
				lf_alert_state_flg=0; // without this , val not stop at 2nd recharge state
			}
		}
		

	}
	// 1min timer (needed for flushing interval)

	if (htim->Instance == TIM14)
	{
		currRTCtimeMins++; // = current_time;
		
		if (day_reset == 1)
		{
			// ## intial value of reset_counter=0
			reset_counter++;
		}
		// ## after day change --> 00:05 --> device restart --> only if pendIrrgFlag=0
		if (reset_counter >= 5)
		{
			NVIC_SystemReset();
		}
		
		if (currRTCtimeMins >= 1440)
		{
			day_reset = 1;
			currRTCtimeMins = 0; //ori 
		}
		
		min_count++;
		
		if (min_count==1)
		{
			min_count=0;
			timer_uart_chk_flg=1;
			
			live_msg_flg=0;	// live pkt enable flag
			
			if((mains_status==1) && (Sim_detected_flag==0))
			{
				no_sim_counter++; // modem reboot --> at 10
			}
			
		}

		minute_counter_mqtt_check++;
		pkt_min_count++; // live pkt timer, 5min and 10min
		if (minute_counter_mqtt_check >= 2)  // 600 seconds = 10 minutes
    {
			LCD_Init_timer_flg=1;
			
      // Reset the counter
      minute_counter_mqtt_check = 0;
      // Call the function every 10 minutes
			//signal_strength_flag=1; // for signal strength display
			minute_counter_mqtt_check_flag = 1;
		}
		
		
		seconds_count++; // new_rr7
		if(seconds_count==1)
		{
			flow_rate1= pulse_count1;  // for flow rate only for live packet
			pulse_count1=0;
			seconds_count=0;
		}
		
 }
}





//mqtt connect
void mqtt_reconnect(){

	HAL_IWDG_Refresh(&hiwdg); // zz7
//	t_var++; // ^^test
	
	keep_alive();
	HAL_Delay(500);
	serverIP(server,port);
	HAL_Delay(4000);

	HAL_IWDG_Refresh(&hiwdg); // zz7
	
	HAL_Delay(500);
	memset(Reply,0,300);
	sub_topic(topic_sub);
	HAL_Delay(1500);
	
}


void WATER_METER_WRITE()
{
	if (PRE_Total_Flow1_count!=Total_Flow1_count)
	{	
		HAL_IWDG_Refresh(&hiwdg); // zz7
		uint8_t *Data;	
		HAL_Delay(200);
		Data = (uint8_t *)&Total_Flow1_count;
		EEPROM_WriteNBytes(EP_WATER_M, Data, 4);
		
		PRE_Total_Flow1_count=Total_Flow1_count;
	}
}

void WATER_METER_RESET_FN()
{
	LCD_Clear();
	sw1=0;
	sw2=0;
	
	MenuDoneCnt = 33;
	
	while (1)
	{
		if (MenuDoneCnt == 33) // FACTORY_RESET() fn
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			factory_reset_flg = ip_no_yes[sw2];
			sw2_max = 2;
			
			LCD_Gotoxy(0, 0);
			LCD_Puts("RESET, SURE?");

			LCD_Gotoxy(0, 1);
			sprintf(msg, " %s                        ", choice_ip[sw2]);
			LCD_Puts(msg);
			HAL_Delay(500);
		}
				
		if ((sw1 == 1) && (input_timeout < 1000) && (factory_reset_flg==2)) //yes
		{
			//water-flow meter reset
			HAL_Delay(500);
			uint8_t *Datta;
		
			Total_Flow1_count=0;
			Datta = (uint8_t *)&Total_Flow1_count;
			EEPROM_WriteNBytes(EP_WATER_M, Datta, 4);
			
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			LCD_Puts("FLOW METER RESET");			
			LCD_Gotoxy(0, 1);
			LCD_Puts("DONE, REBOOTING");
			
			HAL_Delay(1000);
			HAL_NVIC_SystemReset();
		}
		else if ((sw1 == 1) && (input_timeout < 1000) && (factory_reset_flg==1)) //no
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			MenuDoneCnt = 0;
			sw1=0;
			sw2=0;
			break;
		}
	
	} //while end
} // fn end






void reset_lcd_init_timer()
{
	LCD_Init_timer_flg=0;
	minute_counter_mqtt_check = 0;
}
















// pass code input ask --> at the click of cloud setting menu --> tell pass correct or worng --> you can reset passcode if put 0.0.0.0
void PASS_CODE()
{
	pass_flg=1;
	
	LCD_Clear();
	
	sw1=0; 
	sw2=0; 
	sw3=0; 
	swbk=0;
	backflag=0;
	
	MenuDoneCnt=44;
	memset(c_ip, 0x00, sizeof(c_ip));
	oct_1=0;
	oct_2=0;
	oct_3=0;
	oct_4=0;
	
	// do --> read eprom
	
	while(1)
	{
		//HAL_IWDG_Refresh(&hiwdg); // zz7
		
		if (pass_flg==1)
		{	
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			sw2_max = 9; 
			LCD_Gotoxy(0, 0);
			LCD_Puts("ENTER PASSCODE: ");
			LCD_Gotoxy(0, 1);
			sprintf(c_ip, "%01d.%01d.%01d.%01d", oct_1,oct_2,oct_3,oct_4);
			LCD_Puts(c_ip);		
			//HAL_Delay(1000);


			//==oct_1
			if (MenuDoneCnt == 44) // PASS_CODE() fn
			{
				HAL_Delay(600);
				LCD_Gotoxy(0, 1);
				LCD_Puts(" ");
				
				
				if (sw1!=1)
				{
					oct_1 = sw2;
				}
				
				//back -- break\exit from oct-1 --> to pg display
				if (backflag == 1)
				{
					MenuDoneCnt=0;
					sw1=0;
					sw2=0;
					sw3=0;
					swbk=0;
					backflag = 0;
					pass_flg=0;
					break;
				}
					
				if ((sw1 == 1) && (input_timeout < 1000))
				{
					MenuDoneCnt = 2;
					sw1=0;
					sw2=0;
					sw3=0;
					swbk=0;
				}
			}
			
			//==oct_2
			if (MenuDoneCnt == 2)
			{
				HAL_Delay(600);
				LCD_Gotoxy(2, 1);
				LCD_Puts(" ");
				
				if (sw1!=1)
				{
					oct_2 = sw2;
				}
				
					//back oct-1
					if (backflag == 1)
					{
						MenuDoneCnt=44;
						sw1=0;
						sw2=0;
						sw3=0;
						swbk=0;
						backflag = 0;
					}
				
			
				if ((sw1 == 1) && (input_timeout < 1000))
				{
					MenuDoneCnt = 3;	
					sw1=0;
					sw2=0;
					sw3=0;
					swbk=0;
				}
			}
			
			//==oct_3
			if (MenuDoneCnt == 3)
			{
				HAL_Delay(600);
				LCD_Gotoxy(4, 1);
				LCD_Puts(" ");
				
				if (sw1!=1)
				{
					oct_3 = sw2;
				}
				
					//back oct-2
					if (backflag == 1)
					{
						MenuDoneCnt=2;
						sw1=0;
						sw2=0;
						sw3=0;
						swbk=0;
						backflag = 0;
					}
						
				if ((sw1 == 1) && (input_timeout < 1000))
				{
					MenuDoneCnt = 4;
					sw1=0;
					sw2=0;
					sw3=0;
					swbk=0;
				}
			}
			
			//==oct_4
			if (MenuDoneCnt == 4)
			{
				HAL_Delay(600);
				LCD_Gotoxy(6, 1);
				LCD_Puts(" ");
				
				if (sw1!=1)
				{
					oct_4 = sw2;
				}
				
					//back oct-3
					if (backflag == 1)
					{
						MenuDoneCnt=3;
						sw1=0;
						sw2=0;
						sw3=0;
						swbk=0;
						backflag = 0;
					}
				
				if ((sw1 == 1) && (input_timeout < 1000))
				{
					MenuDoneCnt = 5;
					sw1=0;
					sw2=0;
					sw3=0;
					swbk=0;
					pass_flg=0;
				}
			}
		}
		
		if (MenuDoneCnt == 5)
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			//default case
			if (pass_bit_store!=1 && oct_1==pass_ev[0] && oct_2==pass_ev[1] && oct_3==pass_ev[2] && oct_4==pass_ev[3])
			{
				MenuDoneCnt=0;
				sys_fn_flg=1; // to enable/disable 1/0 , the cloud setting menu --> after passcode input
				
				LCD_Clear();
				LCD_Gotoxy(0, 0);
				LCD_Puts("PASS CORRECT");
				HAL_Delay(1000);
				break;
			}
			else if (pass_bit_store==1 && oct_1==pass_ev[0] && oct_2==pass_ev[1] && oct_3==pass_ev[2] && oct_4==pass_ev[3])
			{
				MenuDoneCnt=0;
				sys_fn_flg=1;
				
				LCD_Clear();
				LCD_Gotoxy(0, 0);
				LCD_Puts("PASS CORRECT");
				HAL_Delay(1000);
				break;
			}
			else if (pass_bit_store==1 && oct_1==0 && oct_2==0 && oct_3==0 && oct_4==0) // factory reset --passcode
			{
				HAL_IWDG_Refresh(&hiwdg); // zz7
				
				MenuDoneCnt=0;
				//sys_fn_flg=1;
				
//				LCD_Clear();
//				LCD_Gotoxy(0, 0);
//				LCD_Puts("RESET PASSCODE");
//				HAL_Delay(1000);
				
				SET_PASS_CODE();
				break;
			}
			else
			{
				HAL_IWDG_Refresh(&hiwdg); // zz7
				
				MenuDoneCnt=0;
				sys_fn_flg=0;
				
				LCD_Clear();
				LCD_Gotoxy(0, 0);
				LCD_Puts("PASS WRONG");
				HAL_Delay(2000);
				break;
			}

		}
		
	}// w end
} // fn end 

// to set the pass code value
void SET_PASS_CODE()
{
	uint8_t *Data;
	uint8_t idxs;
	
	pass_flg=1;
	
	LCD_Clear();
	
	sw1=0; 
	sw2=0; 
	sw3=0; 
	swbk=0;
	backflag=0;
	
	MenuDoneCnt=44;
	memset(c_ip, 0x00, sizeof(c_ip));
	oct_1=0;
	oct_2=0;
	oct_3=0;
	oct_4=0;
	
	// do --> read eprom
	
	while(1)
	{
		if (pass_flg==1)
		{	
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			sw2_max = 9; 
			LCD_Gotoxy(0, 0);
			LCD_Puts("RESET PASSCODE: ");
			LCD_Gotoxy(0, 1);
			sprintf(c_ip, "%01d.%01d.%01d.%01d", oct_1,oct_2,oct_3,oct_4);
			LCD_Puts(c_ip);		
			

			//==oct_1
			if (MenuDoneCnt == 44) // PASS_CODE() fn
			{
				HAL_Delay(600);
				LCD_Gotoxy(0, 1);
				LCD_Puts(" ");
				
				if (sw1!=1)
				{
					oct_1 = sw2;
				}
				
				//back -- break\exit from oct-1 --> to pg display
				if (backflag == 1)
				{
					MenuDoneCnt=0;
					sw1=0;
					sw2=0;
					sw3=0;
					swbk=0;
					backflag = 0;
					pass_flg=0;
					break;
				}
					
				if ((sw1 == 1) && (input_timeout < 1000))
				{
					MenuDoneCnt = 2;
					sw1=0;
					sw2=0;
					sw3=0;
					swbk=0;
				}
			}
			
			//==oct_2
			if (MenuDoneCnt == 2)
			{
				HAL_Delay(600);
				LCD_Gotoxy(2, 1);
				LCD_Puts(" ");
				
				if (sw1!=1)
				{
					oct_2 = sw2;
				}
				
					//back oct-1
					if (backflag == 1)
					{
						MenuDoneCnt=44;
						sw1=0;
						sw2=0;
						sw3=0;
						swbk=0;
						backflag = 0;
					}
				
			
				if ((sw1 == 1) && (input_timeout < 1000))
				{
					MenuDoneCnt = 3;	
					sw1=0;
					sw2=0;
					sw3=0;
					swbk=0;
				}
			}
			
			//==oct_3
			if (MenuDoneCnt == 3)
			{
				HAL_Delay(600);
				LCD_Gotoxy(4, 1);
				LCD_Puts(" ");
				
				if (sw1!=1)
				{
					oct_3 = sw2;
				}
				
					//back oct-2
					if (backflag == 1)
					{
						MenuDoneCnt=2;
						sw1=0;
						sw2=0;
						sw3=0;
						swbk=0;
						backflag = 0;
					}
						
				if ((sw1 == 1) && (input_timeout < 1000))
				{
					MenuDoneCnt = 4;
					sw1=0;
					sw2=0;
					sw3=0;
					swbk=0;
				}
			}
			
			//==oct_4
			if (MenuDoneCnt == 4)
			{
				HAL_Delay(600);
				LCD_Gotoxy(6, 1);
				LCD_Puts(" ");
				
				if (sw1!=1)
				{
					oct_4 = sw2;
				}
				
					//back oct-3
					if (backflag == 1)
					{
						MenuDoneCnt=3;
						sw1=0;
						sw2=0;
						sw3=0;
						swbk=0;
						backflag = 0;
					}
				
				if ((sw1 == 1) && (input_timeout < 1000))
				{
					MenuDoneCnt = 5;
					sw1=0;
					sw2=0;
					sw3=0;
					swbk=0;
					pass_flg=0;
				}
			}
		}
		

		if (MenuDoneCnt == 5)
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			// PASS_CODE write
			pass_ev[0]=oct_1;
			pass_ev[1]=oct_2;
			pass_ev[2]=oct_3;
			pass_ev[3]=oct_4;
			
			// to prevent passcode =0000 set 
			entered_pass = pass_ev[0] * 1000 + pass_ev[1] * 100 + pass_ev[2] * 10 + pass_ev[3]; // Combine array elements into a single number
			
			if (entered_pass!=0)
			{
				HAL_IWDG_Refresh(&hiwdg); // zz7
				
				// pass_bit_store write
				pass_bit_store=1;
				Data = &pass_bit_store;
				EEPROM_WriteNBytes(EP_PASS_STORE, Data, 1);
				
				Data = (uint8_t *)&pass_ev[0];
				for (idxs = 0; idxs < 4; idxs++)
				{
					EEPROM_WriteByte(EP_PASS_CODE + idxs, Data[idxs]);
				}
				
				// restart
				LCD_Clear();
				LCD_Gotoxy(0, 0);
				LCD_Puts("PASSCODE SAVED");			
				LCD_Gotoxy(0, 1);
				LCD_Puts("NOW REBOOTING");
				HAL_Delay(2000);

				HAL_NVIC_SystemReset();
			}
			else
			{
				HAL_IWDG_Refresh(&hiwdg); // zz7
				
				MenuDoneCnt = 0;
				sw1=0;
				sw2=0;
				sw3=0;
				swbk=0;
				
				LCD_Clear();
				LCD_Gotoxy(0, 0);
				LCD_Puts("PASSCODE");			
				LCD_Gotoxy(0, 1);
				LCD_Puts("NOT SAVED");
				HAL_Delay(2000);
				break;
			}
		}		
	}// w end
} // fn end 


void data_error(UART_HandleTypeDef *huart)
{
	HAL_IWDG_Refresh(&hiwdg); // zz7
	
		if(((__HAL_UART_GET_FLAG(huart,UART_FLAG_FE)) || (__HAL_UART_GET_FLAG(huart,UART_FLAG_ORE)) || (__HAL_UART_GET_FLAG(huart,UART_FLAG_PE)) || (__HAL_UART_GET_FLAG(huart,UART_FLAG_NE))	)!= RESET)
		{
			__HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
			__HAL_UART_CLEAR_FEFLAG(huart);
			__HAL_UART_CLEAR_OREFLAG(huart);
			__HAL_UART_CLEAR_PEFLAG(huart);
			__HAL_UART_CLEAR_NEFLAG(huart);
		}
}


void test_mode()
{
	LCD_Clear();
	sw1=0;
	sw2=0;
	uint8_t rep_i[4] = {1, 2, 3, 4}; // ##
	char rep_c[4][5] = {"1", "2", "3", "EXIT"}; // ##
	while (1)
	{
		HAL_IWDG_Refresh(&hiwdg); // zz7
		
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
			//MenuDoneCnt = 4; // ori
			sw1=0;
			
			if (rep_v==4)
			{
				HAL_NVIC_SystemReset();
			}
			else
			{
				MenuDoneCnt = 4;
			}
		}
		
		if (MenuDoneCnt == 4)
			while (1)
			{
				HAL_IWDG_Refresh(&hiwdg); // zz7
				
				HAL_GPIO_WritePin(RELAY_PWR_GPIO_Port,RELAY_PWR_Pin,1);
				HAL_Delay(400);
				mcp_return=MCP_PA_WRITE(0x00);
				mcp_return=MCP_PB_WRITE(0x00); 
				HAL_Delay(400);
				
				HAL_IWDG_Refresh(&hiwdg); // zz7
				
				LCD_Gotoxy(0, 0);
				LCD_Puts("START   ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("ON     ");
				HAL_GPIO_WritePin(RELAY_PWR_GPIO_Port,RELAY_PWR_Pin,0);
				HAL_Delay(200);
				
				HAL_GPIO_WritePin(PUMP_R_GPIO_Port,PUMP_R_Pin,1);				
				LCD_Gotoxy(0, 0);
				LCD_Puts("MCP TEST    ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("V1 ON     ");
				HAL_IWDG_Refresh(&hiwdg); // zz7
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7
				HAL_GPIO_WritePin(PUMP_R_GPIO_Port,PUMP_R_Pin,0);
				HAL_Delay(1000);
				
				HAL_IWDG_Refresh(&hiwdg); // zz7
				mcp_init();
				
				mcp_return=MCP_PB_WRITE(0x02);
				LCD_Gotoxy(0, 0);
				LCD_Puts("MCP TEST    ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("V2 ON     ");
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7
				
				mcp_return=MCP_PB_WRITE(0x04);
				LCD_Gotoxy(0, 0);
				LCD_Puts("MCP TEST    ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("V3 ON     ");			
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7
				
				mcp_return=MCP_PB_WRITE(0x08);
				LCD_Gotoxy(0, 0);
				LCD_Puts("MCP TEST    ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("V4 ON     ");
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7

				
				mcp_return=MCP_PB_WRITE(0x10);
				LCD_Gotoxy(0, 0);
				LCD_Puts("MCP TEST    ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("V5 ON     ");
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7
				
				
				mcp_return=MCP_PB_WRITE(0x20);
				LCD_Gotoxy(0, 0);
				LCD_Puts("MCP TEST  ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("V6 ON     ");
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7
				
				
		
				mcp_return=MCP_PB_WRITE(0x40);
				LCD_Gotoxy(0, 0);
				LCD_Puts("MCP TEST  ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("V7 ON     ");
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7
				
				mcp_return=MCP_PB_WRITE(0x00); 
				HAL_Delay(2000);
				HAL_IWDG_Refresh(&hiwdg); // zz7

				//==A block
				mcp_return=MCP_PA_WRITE(0x20);
				LCD_Gotoxy(0, 0);
				LCD_Puts("MCP TEST    ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("V8 ON     ");
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7

				mcp_return=MCP_PA_WRITE(0x40);
				LCD_Gotoxy(0, 0);
				LCD_Puts("MCP TEST    ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("V9 ON     ");
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7
				

				mcp_return=MCP_PA_WRITE(0x80);
				LCD_Gotoxy(0, 0);
				LCD_Puts("MCP TEST    ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("V10 ON     ");
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7
				

				mcp_return=MCP_PA_WRITE(0x10);
				LCD_Gotoxy(0, 0);
				LCD_Puts("MCP TEST    ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("V11 ON     ");
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7
				

				mcp_return=MCP_PA_WRITE(0x08);
				LCD_Gotoxy(0, 0);
				LCD_Puts("MCP TEST    ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("V12 ON     ");
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7
				

				mcp_return=MCP_PA_WRITE(0x01);
				LCD_Gotoxy(0, 0);
				LCD_Puts("MCP TEST    ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("V13 ON     ");
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7
				

				mcp_return=MCP_PA_WRITE(0x02);
				LCD_Gotoxy(0, 0);
				LCD_Puts("MCP TEST    ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("V14 ON     ");
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7
				

				mcp_return=MCP_PA_WRITE(0x04);
				LCD_Gotoxy(0, 0);
				LCD_Puts("MCP TEST    ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("V15 ON     ");
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7

				mcp_return=MCP_PA_WRITE(0x00);
				LCD_Gotoxy(0, 0);
				LCD_Puts("MCP TEST    ");
				LCD_Gotoxy(0, 1);
				LCD_Puts("off     ");
				HAL_Delay(5000);
				HAL_IWDG_Refresh(&hiwdg); // zz7
				
				rep_stop++;
	
				if (rep_v == rep_stop)
				{
					MenuDoneCnt = 0;
					sw1=0;
					sw2=0;
					rep_stop=0;
					HAL_GPIO_WritePin(RELAY_PWR_GPIO_Port,RELAY_PWR_Pin,1);
					HAL_GPIO_WritePin(PUMP_R_GPIO_Port,PUMP_R_Pin,0);
					LCD_Clear();
					break;
				}
			}		
	}
}



void ipSet()
{
	uint8_t *Datta;
	uint8_t iidx;
	
	LCD_Clear();
	
	sw1=0;
	sw2=0;
	sw3=0;
	swbk=0;
	backflag=0; 
	
	ip_setup_w_loop=1;

	MenuDoneCnt=22;

	while (ip_setup_w_loop==1)
	{

		if (MenuDoneCnt == 22) //ipSet fn
		{	
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			ip_save_flg = ip_no_yes[sw2];
			sw2_max = 2;
			
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			LCD_Puts("CHANGE IP, SURE?");
			LCD_Gotoxy(0, 1);
			sprintf(msg, " %s                        ", choice_ip[sw2]);
			LCD_Puts(msg);
			HAL_Delay(500);
		}
		
		// yes -- choice_ip[sw2]
		if ((sw1 == 1) && (input_timeout < 1000) && ip_save_flg==2)
		{
			
			ip_save_flg=0;
			ip_setup=1;
			
			LCD_Clear();
			memset(c_ip, 0x00, sizeof(c_ip));
			oct_1=0;
			oct_2=0;
			oct_3=0;
			oct_4=0;
			
			sw5_ip_flg=1; // to enable button-5 for this fn 
			
			MenuDoneCnt=1;
			sw1=0;
			sw2=0;
			sw3=0;
			swbk=0;
		}
		else if ((sw1 == 1) && (input_timeout < 1000) && ip_save_flg==1) // no, choice_ip[sw2]
		{
			ip_save_flg=0;
			sw1=0;
			break;
		}
		
		
//===============	
		if (ip_setup==1)
		{	
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			sw2_max = 255; // req, oth-w sw2 not work
			LCD_Gotoxy(0, 0);
			LCD_Puts("SET IP: ");
			LCD_Gotoxy(0, 1);
			sprintf(c_ip, "%03d.%03d.%03d.%03d", oct_1,oct_2,oct_3,oct_4);
			LCD_Puts(c_ip);		
			//HAL_Delay(1000);

			// to correct the sw2 behaviour due to sw5 -- sw2=254+10
			if (sw2>=255)
			{
				sw1=0;
				sw2=0;
				sw3=0;
				swbk=0;
			}
	
			//==oct_1
			if (MenuDoneCnt == 1)
			{
				HAL_Delay(600);
				LCD_Gotoxy(0, 1);
				LCD_Puts("   ");
				
				
				if (sw1!=1)
				{
					oct_1 = sw2;
				}
				
				//back -- break\exit from oct-1 --> to cloud_set_menu()
				if (backflag == 1)
				{
					MenuDoneCnt=0;
					sw1=0;
					sw2=0;
					sw3=0;
					swbk=0;
					backflag = 0;
					ip_setup=0;
					break;
				}
					
				if ((sw1 == 1) && (input_timeout < 1000))
				{
					MenuDoneCnt = 2;
					sw1=0;
					sw2=0;
					sw3=0;
					swbk=0;
				}
			}
			
			//==oct_2
			if (MenuDoneCnt == 2)
			{
				HAL_Delay(600);
				LCD_Gotoxy(4, 1);
				LCD_Puts("   ");
				
				if (sw1!=1)
				{
					oct_2 = sw2;
				}
				
					//back oct-1
					if (backflag == 1)
					{
						MenuDoneCnt=1;
						sw1=0;
						sw2=0;
						sw3=0;
						swbk=0;
						backflag = 0;
					}
				
			
				if ((sw1 == 1) && (input_timeout < 1000))
				{
					MenuDoneCnt = 3;	
					sw1=0;
					sw2=0;
					sw3=0;
					swbk=0;
				}
			}
			
			//==oct_3
			if (MenuDoneCnt == 3)
			{
				HAL_Delay(600);
				LCD_Gotoxy(8, 1);
				LCD_Puts("   ");
				
				if (sw1!=1)
				{
					oct_3 = sw2;
				}
				
					//back oct-2
					if (backflag == 1)
					{
						MenuDoneCnt=2;
						sw1=0;
						sw2=0;
						sw3=0;
						swbk=0;
						backflag = 0;
					}
						
				if ((sw1 == 1) && (input_timeout < 1000))
				{
					MenuDoneCnt = 4;
					sw1=0;
					sw2=0;
					sw3=0;
					swbk=0;
				}
			}
			
			//==oct_4
			if (MenuDoneCnt == 4)
			{
				HAL_Delay(600);
				LCD_Gotoxy(12, 1);
				LCD_Puts("   ");
				
				if (sw1!=1)
				{
					oct_4 = sw2;
				}
				
					//back oct-3
					if (backflag == 1)
					{
						MenuDoneCnt=3;
						sw1=0;
						sw2=0;
						sw3=0;
						swbk=0;
						backflag = 0;
					}
				
				if ((sw1 == 1) && (input_timeout < 1000))
				{
					MenuDoneCnt = 5;
					sw1=0;
					sw2=0;
					sw3=0;
					swbk=0;
					ip_setup=0; // to stop this --> LCD_Puts(c_ip); -- and enter into -- if (MenuDoneCnt == 5)
					sw5_ip_flg=0; // // to disable button-5 for this fn 
				}
			}
		}
	
		
		if (MenuDoneCnt == 5)
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			ip_save_flg = ip_no_yes[sw2];
			sw2_max = 2;
			LCD_Clear();
			
			LCD_Gotoxy(0, 0);
			LCD_Puts("SAVE IP, SURE?");

			LCD_Gotoxy(0, 1);
			sprintf(msg, " %s                        ", choice[sw2]);
			LCD_Puts(msg);
			HAL_Delay(500);
		}
		
		if ((sw1 == 1) && (input_timeout < 1000) && ip_save_flg==1)
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			memset(server, 0x00, sizeof(server));
			HAL_Delay(200);
			sprintf(server, "%d.%d.%d.%d", oct_1,oct_2,oct_3,oct_4);
			sw1=0;
			sw5_ip_flg=0; 
			ip_setup=0; 
	
			// eprom write ip bit
			IP_store_bit=1;
			Datta = &IP_store_bit;
			EEPROM_WriteNBytes(EP_IP_STORE, Datta, 1);
			
			// eprom write ip 
			Datta = (uint8_t *)&server[0];
			for (iidx = 0; iidx < 16; iidx++)
			{
				EEPROM_WriteByte(EP_server_IP + iidx, Datta[iidx]); // Error Flag
			}
		
			//restart
			HAL_Delay(800);
			HAL_NVIC_SystemReset();
		}
		else if ((sw1 == 1) && (input_timeout < 1000) && ip_save_flg==2)
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			sw1=0;
			sw5_ip_flg=0;
			ip_setup=0; // to again config
			MenuDoneCnt=0;
			break; //cancel
		}
	}
}


// set port fn
// max port num 65535
void setPort()
{
	uint8_t *Datta;
	uint8_t iidx;
	
	LCD_Clear();
	
	sw1=0;
	sw2=0;
	sw3=0;
	swbk=0;
	backflag=0; 
	
	ip_setup_w_loop=1;

	MenuDoneCnt=22;

	while (ip_setup_w_loop==1)
	{
		
		if (MenuDoneCnt == 22) //ipSet fn
		{	
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			ip_save_flg = ip_no_yes[sw2];
			sw2_max = 2;
			
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			LCD_Puts("CHG PORT, SURE?");
			LCD_Gotoxy(0, 1);
			sprintf(msg, " %s                        ", choice_ip[sw2]);
			LCD_Puts(msg);
			HAL_Delay(500);
		}
		
		// yes -- choice_ip[sw2]
		if ((sw1 == 1) && (input_timeout < 1000) && ip_save_flg==2)
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			ip_save_flg=0;
			ip_setup=1;
			
			LCD_Clear();
			memset(c_ip, 0x00, sizeof(c_ip));
			oct_1=0;
			oct_2=0;
			oct_3=0;
			oct_4=0;
			
			//sw5_ip_flg=1; // to enable button-5 for this fn 
			
			MenuDoneCnt=1;
			sw1=0;
			sw2=0;
			sw3=0;
			swbk=0;
		}
		else if ((sw1 == 1) && (input_timeout < 1000) && ip_save_flg==1) // no, choice_ip[sw2]
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			ip_save_flg=0;
			sw1=0;
			break;
		}
		
		
//===============	
		if (ip_setup==1)
		{	
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			//sw2_max = 100; // not req, for port
			LCD_Gotoxy(0, 0);
			LCD_Puts("SET PORT NUM: ");
			LCD_Gotoxy(0, 1);
			sprintf(c_ip, "%01d.%01d.%01d.%01d", oct_1,oct_2,oct_3,oct_4);
			LCD_Puts(c_ip);		
			//HAL_Delay(1000);


	
			//==oct_1
			if (MenuDoneCnt == 1)
			{
				sw2_max = 10;
				
				HAL_Delay(600);
				LCD_Gotoxy(0, 1);
				LCD_Puts(" ");
				
				
				if (sw1!=1)
				{
					oct_1 = sw2;
				}
				
				//back -- break\exit from oct-1 --> to cloud_set_menu()
				if (backflag == 1)
				{
					MenuDoneCnt=0;
					sw1=0;
					sw2=0;
					sw3=0;
					swbk=0;
					backflag = 0;
					ip_setup=0;
					break;
				}
					
				if ((sw1 == 1) && (input_timeout < 1000))
				{
					MenuDoneCnt = 2;
					sw1=0;
					sw2=0;
					sw3=0;
					swbk=0;
				}
			}
			
			//==oct_2
			if (MenuDoneCnt == 2)
			{
				sw2_max = 10;
				
				HAL_Delay(600);
				LCD_Gotoxy(2, 1);
				LCD_Puts(" ");
				
				if (sw1!=1)
				{
					oct_2 = sw2;
				}
				
					//back oct-1
					if (backflag == 1)
					{
						MenuDoneCnt=1;
						sw1=0;
						sw2=0;
						sw3=0;
						swbk=0;
						backflag = 0;
					}
				
			
				if ((sw1 == 1) && (input_timeout < 1000))
				{
					MenuDoneCnt = 3;	
					sw1=0;
					sw2=0;
					sw3=0;
					swbk=0;
				}
			}
			
			//==oct_3
			if (MenuDoneCnt == 3)
			{
				sw2_max = 10;
				
				HAL_Delay(600);
				LCD_Gotoxy(4, 1);
				LCD_Puts(" ");
				
				if (sw1!=1)
				{
					oct_3 = sw2;
				}
				
					//back oct-2
					if (backflag == 1)
					{
						MenuDoneCnt=2;
						sw1=0;
						sw2=0;
						sw3=0;
						swbk=0;
						backflag = 0;
					}
						
				if ((sw1 == 1) && (input_timeout < 1000))
				{
					MenuDoneCnt = 4;
					sw1=0;
					sw2=0;
					sw3=0;
					swbk=0;
				}
			}
			
			//==oct_4
			if (MenuDoneCnt == 4)
			{
				sw2_max = 100;
				
				HAL_Delay(600);
				LCD_Gotoxy(6, 1);
				LCD_Puts("  ");
				
				if (sw1!=1)
				{
					oct_4 = sw2;
				}
				
					//back oct-3
					if (backflag == 1)
					{
						MenuDoneCnt=3;
						sw1=0;
						sw2=0;
						sw3=0;
						swbk=0;
						backflag = 0;
					}
				
				if ((sw1 == 1) && (input_timeout < 1000))
				{
					MenuDoneCnt = 5;
					sw1=0;
					sw2=0;
					sw3=0;
					swbk=0;
					ip_setup=0; // to stop this --> LCD_Puts(c_ip); -- and enter into -- if (MenuDoneCnt == 5)
					sw5_ip_flg=0; // // to disable button-5 for this fn 
				}
			}
		}
	
		
		if (MenuDoneCnt == 5)
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			ip_save_flg = ip_no_yes[sw2];
			sw2_max = 2;
			LCD_Clear();
			
			LCD_Gotoxy(0, 0);
			LCD_Puts("SAVE PORT, SURE?");

			LCD_Gotoxy(0, 1);
			sprintf(msg, " %s                        ", choice[sw2]);
			LCD_Puts(msg);
			HAL_Delay(500);
		}
		
		if ((sw1 == 1) && (input_timeout < 1000) && ip_save_flg==1)
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			memset(port, 0x00, sizeof(port));
			HAL_Delay(200);
			sprintf(port, "%d%d%d%d", oct_1,oct_2,oct_3,oct_4);
			sw1=0;
			sw5_ip_flg=0;
			ip_setup=0;
	
			// eprom write port bit
			PORT_store_bit=1;
			Datta = &PORT_store_bit;
			EEPROM_WriteNBytes(EP_PORT_STORE, Datta, 1);
			
			// eprom write port 
			Datta = (uint8_t *)&port[0];
			for (iidx = 0; iidx < 6; iidx++)
			{
				EEPROM_WriteByte(EP_PORT_NUM + iidx, Datta[iidx]);
			}
			
			//restart
			HAL_Delay(800);
			HAL_NVIC_SystemReset();
		}
		else if ((sw1 == 1) && (input_timeout < 1000) && ip_save_flg==2)
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			sw1=0;
			sw5_ip_flg=0;
			ip_setup=0; // to again config
			MenuDoneCnt=0;
			break; //cancel
		}
	}
}


void FACTORY_RESET()
{
	LCD_Clear();
	sw1=0;
	sw2=0;
	
	MenuDoneCnt = 33;
	
	while (1)
	{
		if (MenuDoneCnt == 33) // FACTORY_RESET() fn
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			factory_reset_flg = ip_no_yes[sw2];
			sw2_max = 2;
			
			LCD_Gotoxy(0, 0);
			LCD_Puts("ERASE, SURE?");

			LCD_Gotoxy(0, 1);
			sprintf(msg, " %s                        ", choice_ip[sw2]);
			LCD_Puts(msg);
			HAL_Delay(500);
		}
				
		if ((sw1 == 1) && (input_timeout < 1000) && (factory_reset_flg==2)) //yes
		{
			HAL_Delay(1000);
			uint16_t v_eepromAddress_u16;
			
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			for (v_eepromAddress_u16 = EP_MASTER_PUMP_SCHD_STTIME; v_eepromAddress_u16 < 1099; v_eepromAddress_u16++)
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
			
			HAL_Delay(500);
			HAL_NVIC_SystemReset();
		}
		else if ((sw1 == 1) && (input_timeout < 1000) && (factory_reset_flg==1)) //no
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			MenuDoneCnt = 0;
			sw1=0;
			sw2=0;
			break;
		}
	
	} //while end
} // fn end




void APN_SET_REMOTE(char *apn_str_value) 
	{
		HAL_IWDG_Refresh(&hiwdg); // zz7
		
  	uint8_t *DData;
		uint8_t iddx;	  
	
		// Copy the new value 
    strncpy(apn, apn_str_value, sizeof(apn) - 1);
    apn[sizeof(apn) - 1] = '\0'; // Ensuring null termination
	
		// eprom apn bit write
		APN_store_bit=1;
		DData = &APN_store_bit;
		EEPROM_WriteNBytes(EP_APN_STORE, DData, 1);
		
		HAL_IWDG_Refresh(&hiwdg); // zz7
		
		// eprom apn write
		DData = (uint8_t *)&apn[0];
		for (iddx = 0; iddx < 65; iddx++)
		{
			EEPROM_WriteByte(EP_apn + iddx, DData[iddx]);
		}
		
		HAL_IWDG_Refresh(&hiwdg); // zz7
		
		HAL_GPIO_WritePin(RELAY_PWR_GPIO_Port,RELAY_PWR_Pin,1);
		HAL_GPIO_WritePin(PUMP_R_GPIO_Port,PUMP_R_Pin,0);

		//valve_output(&gvCommand,&gvCommand_pcf);  // %% chk_imp
		
		HAL_IWDG_Refresh(&hiwdg); // zz7
		
		HAL_Delay(500);
		LCD_Clear();
		LCD_Gotoxy(0, 0);
		LCD_Puts("APN SET DONE");			
		LCD_Gotoxy(0, 1);
		LCD_Puts("NOW REBOOTING");
		
		apn_msg_confirmed();
		HAL_Delay(2000);
		
		HAL_NVIC_SystemReset();
}

void APN_SET_FN(char *apn_str_value) {
    // Copy the new value 
    strncpy(apn, apn_str_value, sizeof(apn) - 1);
    apn[sizeof(apn) - 1] = '\0'; // Ensuring null termination
}

//#if 0
void apn_set_menu()
{
	uint8_t *DData;
	uint8_t iddx;
	MenuDoneCnt = 0;
	
	HAL_Delay(1000);
	LCD_Clear();
	sw1=0;
	sw2=0;
	sw3=0;
	swbk=0;
	backflag = 0;
	uint8_t rep_i[4] = {1, 2, 3, 4}; // ##
	char rep_c[4][16] = {"1. AIRTEL", "2. JIO", "3. IDEA", "4. BSNL"}; // ##
	while (1)
	{
		HAL_IWDG_Refresh(&hiwdg); // zz7
	
		rep_v = rep_i[sw2];
		sw2_max = 4;
		LCD_Gotoxy(0, 0);
		LCD_Puts("SELECT APN: ");
		LCD_Gotoxy(0, 1);
		sprintf(msg, "%s     ", rep_c[sw2]);
		LCD_Puts(msg);		

		//back -- break\exit --> to pg display
		if (backflag == 1)
		{
			MenuDoneCnt=0;
			sw1=0;
			sw2=0;
			sw3=0;
			swbk=0;
			backflag = 0;
			break;
		}
		
		if ((sw1 == 1) && (input_timeout < 1000))
		{
			MenuDoneCnt = 5;
			sw1=0;
		}
		
		if (MenuDoneCnt == 5)
		{
		
			if (rep_v==1)
			{
				APN_SET_FN("airtelgprs.com");
			}
			else if (rep_v==2)
			{
				APN_SET_FN("jionet");
			}
			else if (rep_v==3)
			{
				APN_SET_FN("www");
			}
			else if (rep_v==4)
			{
				APN_SET_FN("bsnlnet");
			}
			
			MenuDoneCnt = 0;
			
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			// eprom apn bit write
			APN_store_bit=1;
			DData = &APN_store_bit;
			EEPROM_WriteNBytes(EP_APN_STORE, DData, 1);
			
			// eprom apn write
			DData = (uint8_t *)&apn[0];
			for (iddx = 0; iddx < 65; iddx++)
			{
				EEPROM_WriteByte(EP_apn + iddx, DData[iddx]);
			}
			
		
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			LCD_Puts("APN SET DONE");			
			LCD_Gotoxy(0, 1);
			LCD_Puts("NOW REBOOTING");
			HAL_Delay(2000);
			
			HAL_NVIC_SystemReset();
			//break;
		}
		
	}
}
//#endif


void cloud_set_menu()
{	
	MenuDoneCnt = 0;
	
	LCD_Clear();
	sw1=0;
	sw2=0;
	uint8_t sys_i[7] = {1, 2, 3, 4, 5, 6, 7}; // ##
	char sys_c[7][16] = {"1. SET APN", "2. SET IP", "3. SET PORT", "4. FACTORY RESET", "5. SET PASSCODE", "6. FLOW M RESET", "7. CANCEL"}; // ##
	while (1)
	{
		HAL_IWDG_Refresh(&hiwdg); // zz7
		
		LCD_Clear();
		m_id = sys_i[sw2];
		sw2_max = 7;
		LCD_Gotoxy(0, 0);
		LCD_Puts("CLOUD MENU: ");
		LCD_Gotoxy(0, 1);
		sprintf(msg, "%s     ", sys_c[sw2]);
		LCD_Puts(msg);		
		HAL_Delay(500);

		if ((sw1 == 1) && (input_timeout < 1000))
		{
			MenuDoneCnt = 5;
			sw1=0;
			sw2=0;
		}
		
		if (MenuDoneCnt == 5)
		{
			if (m_id==1)
			{
				// apn set
				apn_set_menu();
			}
			else if (m_id==2)
			{
				//APN_SET_FN("jionet");
				ipSet();
			}
			else if (m_id==3)
			{
				//set port
				setPort();
			}
			else if (m_id==4)
			{
				//factory reset
				FACTORY_RESET();
			}
			else if (m_id==5)
			{
				//set passcode
				SET_PASS_CODE();
			}
			else if (m_id==6)
			{
				//water-flow meter reset
				WATER_METER_RESET_FN();
			}
			else if (m_id==7)
			{
				//cancel
				break;
			}
			
			//MenuDoneCnt = 0;
			
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
		HAL_IWDG_Refresh(&hiwdg); // zz7
		
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
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
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
				HAL_IWDG_Refresh(&hiwdg); // zz7

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
				HAL_IWDG_Refresh(&hiwdg); // zz7
				
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
				HAL_IWDG_Refresh(&hiwdg); // zz7
				
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
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
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
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
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
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
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
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
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
	HAL_IWDG_Refresh(&hiwdg); // zz7
	
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
	HAL_IWDG_Refresh(&hiwdg); // zz7
	
	int hrs, mins;
	LCD_Clear();
	RTC_TimeTypeDef getTime = {0};
	RTC_DateTypeDef getDate = {0};
	hrs = currRTCtimeMins/60;
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
	//HAL_Delay(1000); // ori
	HAL_Delay(100);
	LCD_Gotoxy(8, 1);
	LCD_Puts(" ");
	HAL_Delay(100);
	LCD_Gotoxy(0, 1);
	sprintf(lcd_data, "TIME: %02d:%02d  %3s", hrs % 24, mins, week[getDate.WeekDay - 1]);
	LCD_Puts(lcd_data);
	//HAL_Delay(1000); // ori
	HAL_Delay(100);
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

}




void cloudSetting()
{
	HAL_IWDG_Refresh(&hiwdg); // zz7
	
	if (IP_store_bit==1)
	{
		LCD_Clear();
		LCD_Gotoxy(0, 0);
		LCD_Puts("SERVER IP : ");
		LCD_Gotoxy(0, 1);
		sprintf(lcd_data, "%s", server);
		LCD_Puts(lcd_data);
		HAL_Delay(4000);
	}
	else 
	{
		LCD_Clear();
		LCD_Gotoxy(0, 0);
		LCD_Puts("DEFAULT IP: ");
		LCD_Gotoxy(0, 1);
		sprintf(lcd_data, "%s", server);
		LCD_Puts(lcd_data);
		HAL_Delay(4000);
	}
	
	HAL_IWDG_Refresh(&hiwdg); // zz7
	
	if (APN_store_bit==1)
	{
		LCD_Clear();
		LCD_Gotoxy(0, 0);
		LCD_Puts("APN : ");
		
		uint8_t apn_length = strlen(apn);

		
		if (apn_length>=17)
		{
			char scroll_buffer[17] = {0}; // 16 chars + null terminator
			uint8_t apn_length = strlen(apn);
		
			for (int scroll_pos = 0; scroll_pos <= apn_length - 16; scroll_pos++) 
			{
				HAL_IWDG_Refresh(&hiwdg); // zz7
        for (int i = 0; i < 16; i++) 
				{
						HAL_IWDG_Refresh(&hiwdg); // zz7
            scroll_buffer[i] = apn[scroll_pos + i];
        }
        scroll_buffer[16] = '\0';

        LCD_Gotoxy(0, 1);
        LCD_Puts(scroll_buffer);

        HAL_Delay(1000); // Adjust scrolling speed
			}
		}
		else
		{
				HAL_IWDG_Refresh(&hiwdg); // zz7
				LCD_Gotoxy(0, 1);
				LCD_Puts(apn);
				HAL_Delay(3000);
		}
		
		HAL_Delay(1000);
		
	}
	else
	{
		HAL_IWDG_Refresh(&hiwdg); // zz7
		LCD_Clear();
		LCD_Gotoxy(0, 0);
		LCD_Puts("DEFAULT APN : ");
		LCD_Gotoxy(0, 1);
		sprintf(lcd_data, "%s", apn);
		LCD_Puts(lcd_data);
		HAL_Delay(4000);
	}
	
	HAL_IWDG_Refresh(&hiwdg); // zz7
	
	if (PORT_store_bit==1)
	{
		//==port
		LCD_Clear();
		LCD_Gotoxy(0, 0);
		LCD_Puts("PORT : ");
		LCD_Gotoxy(0, 1);
		sprintf(lcd_data, "%s", port);
		LCD_Puts(lcd_data);
		HAL_Delay(4000);
	}
	else
	{
		//==port
		LCD_Clear();
		LCD_Gotoxy(0, 0);
		LCD_Puts("DEFAULT PORT : ");
		LCD_Gotoxy(0, 1);
		sprintf(lcd_data, "%s", port);
		LCD_Puts(lcd_data);
		HAL_Delay(4000);
	}
	
	HAL_IWDG_Refresh(&hiwdg); // zz7
	
	LCD_Clear();
	LCD_Gotoxy(0, 0);
	LCD_Puts("IMEI");
	LCD_Gotoxy(0, 1);
	LCD_Puts(imei_id);
	HAL_Delay(4000);
	
	HAL_IWDG_Refresh(&hiwdg); // zz7
	
	memset(lcd_data, 0x00, sizeof(lcd_data));
	LCD_Clear();
	LCD_Gotoxy(0, 0);
	LCD_Puts("LIVE PKT TIME:");
	LCD_Gotoxy(0, 1);
	sprintf(lcd_data, "%u MIN", custom_pkt_time);
	LCD_Puts(lcd_data);
	HAL_Delay(4000);
	
	HAL_IWDG_Refresh(&hiwdg); // zz7
	
	LCD_Clear();
	LCD_Gotoxy(0, 0);
	LCD_Puts("FIRMWARE VER");
	LCD_Gotoxy(0, 1);
	LCD_Puts("1.0");
	HAL_Delay(4000);
	
	HAL_IWDG_Refresh(&hiwdg); // zz7
	
	if(rain_read_disable_flg==1)
	{
		LCD_Clear();
		LCD_Gotoxy(0, 0);
		LCD_Puts("RAIN SENSOR");
		LCD_Gotoxy(0, 1);
		LCD_Puts("DISABLE");
		HAL_Delay(4000);
	}
	else
	{
		LCD_Clear();
		LCD_Gotoxy(0, 0);
		LCD_Puts("RAIN SENSOR");
		LCD_Gotoxy(0, 1);
		LCD_Puts("ENABLE");
		HAL_Delay(4000);
	}
	
	HAL_IWDG_Refresh(&hiwdg); // zz7
	
	if(lf_read_disable_flg==1)
	{
		LCD_Clear();
		LCD_Gotoxy(0, 0);
		LCD_Puts("LF SENSOR");
		LCD_Gotoxy(0, 1);
		LCD_Puts("DISABLE");
		HAL_Delay(4000);
	}
	else
	{
		LCD_Clear();
		LCD_Gotoxy(0, 0);
		LCD_Puts("LF SENSOR");
		LCD_Gotoxy(0, 1);
		LCD_Puts("ENABLE");
		HAL_Delay(4000);
	}
	
	HAL_IWDG_Refresh(&hiwdg); // zz7
	
	memset(lcd_data, 0x00, sizeof(lcd_data));
	LCD_Clear();
	LCD_Gotoxy(0, 0);
	LCD_Puts("PIT TIME:");
	LCD_Gotoxy(0, 1);
	sprintf(lcd_data, "%u SEC", pumpInitiationTimeSecs);
	LCD_Puts(lcd_data);
	HAL_Delay(4000);
	memset(lcd_data, 0x00, sizeof(lcd_data));
	
	HAL_IWDG_Refresh(&hiwdg); // zz7
	
}




void pub_msg_sch()
{


	//if ((LCD_Init_timer_flg==1 && lcd_on_flag == 1) || lcd_prob==1)
	if ((LCD_Init_timer_flg==1 && mains_status == 1) || lcd_prob==1)
	{
		HAL_IWDG_Refresh(&hiwdg); // zz7
		LCD_Init();
		lcd_on();
		lcd_prob=0;
		////&&**live_values();
		LCD_Init_timer_flg=0;
	}
	
	if (currRTCtimeMins==1438 && flow_pulse_store_flg==0)
	{
		flow_pulse_store_flg=1;
		uint8_t *Data;
		Data = &TotalFlow1Counter;
		EEPROM_WriteNBytes(EP_FLOW_PULSE, Data, 1);
	}
	else if (currRTCtimeMins!=1438 && flow_pulse_store_flg==1)
	{
		flow_pulse_store_flg=0;
	}
	
	if ((water_flg==1) && (mains_status==1))
	{
		HAL_IWDG_Refresh(&hiwdg); // zz7
		
		WATER_METER_WRITE();
		water_flg=0;
	}
	
	// after every status change -- live pkt send
	if (status_chg_flg==1)
	{
		HAL_IWDG_Refresh(&hiwdg); // zz7
		
		pub_live_msg();	
		
		status_chg_flg=0;
	
		live_msg_flg=1;  
		HAL_Delay(300);
	}
	
	if (pkt_min_count==0) // 255 to 0 --> 0%5=0, 0%10=0
	{
		pkt_min_count=1;
		HAL_Delay(100);
	}
	
	// for every 3 min --- live_msg 
	//if ((pkt_min_count%custom_pkt_time)==0 && live_msg_flg==0)
	if ((custom_pkt_time!= 0) && (pkt_min_count%custom_pkt_time)==0 && (live_msg_flg==0))
	{
		HAL_IWDG_Refresh(&hiwdg); // zz7
		HAL_Delay(100);
	
		pub_live_msg();
		live_msg_flg=1;	
	}
	
	if (no_sim_counter>=30)
	{
		no_sim_counter=0;
		Sim_detected_flag=1;
		
		modem_reboot_count_on_mains=1; // for reconnecting , from 2 to 1 , --> modem reboot only one times
		reconnect_cn=0; // value append in 2min timer -- so after main_on --> assign 0
	}
	
	RTC_TimeShow1();
	HAL_Delay(300);
}

//======= rain disable fn
void RAIN_DISABLE()
{
	LCD_Clear();
	sw1=0;
	sw2=0;
	
	MenuDoneCnt = 33;
	
	while (1)
	{
		if (MenuDoneCnt == 33) // FACTORY_RESET() fn
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			factory_reset_flg = ip_no_yes[sw2];
			sw2_max = 2;
			
			LCD_Gotoxy(0, 0);
			LCD_Puts("DISABLE, SURE?");

			LCD_Gotoxy(0, 1);
			sprintf(msg, " %s                        ", choice_ip[sw2]);
			LCD_Puts(msg);
			HAL_Delay(500);
		}
				
		if ((sw1 == 1) && (input_timeout < 1000) && (factory_reset_flg==2)) //yes
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			// eprom rain bit write
			HAL_Delay(500);
			
			uint8_t *DData;
			
			rain_read_disable_flg=1;
			DData = &rain_read_disable_flg;
			EEPROM_WriteNBytes(EP_RAIN_DISABLE_BIT, DData, 1);
			
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			LCD_Puts("RAIN DISABLE");			
			LCD_Gotoxy(0, 1);
			LCD_Puts("NOW REBOOTING");
			HAL_Delay(2000);
			
			HAL_NVIC_SystemReset();
		}
		else if ((sw1 == 1) && (input_timeout < 1000) && (factory_reset_flg==1)) //no
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			MenuDoneCnt = 0;
			sw1=0;
			sw2=0;
			break;
		}
	
	} //while end
} // fn end

//===== low flow disable fn
void LF_DISABLE()
{
	LCD_Clear();
	sw1=0;
	sw2=0;
	
	MenuDoneCnt = 33;
	
	while (1)
	{
		if (MenuDoneCnt == 33) // FACTORY_RESET() fn
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			factory_reset_flg = ip_no_yes[sw2];
			sw2_max = 2;
			
			LCD_Gotoxy(0, 0);
			LCD_Puts("DISABLE, SURE?");

			LCD_Gotoxy(0, 1);
			sprintf(msg, " %s                        ", choice_ip[sw2]);
			LCD_Puts(msg);
			HAL_Delay(500);
		}
				
		if ((sw1 == 1) && (input_timeout < 1000) && (factory_reset_flg==2)) //yes
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			// eprom LF bit write
			HAL_Delay(500);
			
			uint8_t *DData;
			
			lf_read_disable_flg=1;
			DData = &lf_read_disable_flg;
			EEPROM_WriteNBytes(EP_LF_DISABLE_BIT, DData, 1);
			
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			LCD_Puts("LF DISABLE");			
			LCD_Gotoxy(0, 1);
			LCD_Puts("NOW REBOOTING");
			HAL_Delay(2000);	
			
			HAL_NVIC_SystemReset();
		}
		else if ((sw1 == 1) && (input_timeout < 1000) && (factory_reset_flg==1)) //no
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			MenuDoneCnt = 0;
			sw1=0;
			sw2=0;
			break;
		}
	
	} //while end
} // fn end

void sensor_setting()
{	
	MenuDoneCnt = 0;
	
	LCD_Clear();
	sw1=0;
	sw2=0;
	backflag=0;
	
	uint8_t syss_i[5] = {1, 2, 3, 4, 5}; // ##	
	char syss_c[5][16] = {"1. RAIN DISABLE","2. LF DISABLE","3. RAIN ENABLE","4. LF ENABLE","5. EXIT"};
	while (1)
	{
		HAL_IWDG_Refresh(&hiwdg); // zz7
		
		LCD_Clear();
		dig_id = syss_i[sw2];
		sw2_max = 5;
		LCD_Gotoxy(0, 0);
		LCD_Puts("SENSOR:");
		LCD_Gotoxy(0, 1);
		sprintf(msg, "%s     ", syss_c[sw2]);
		LCD_Puts(msg);		
		HAL_Delay(500);

		if ((sw1 == 1) && (input_timeout < 1000))
		{
			MenuDoneCnt = 5;
			sw1=0;
			sw2=0;
		}
		
		if (MenuDoneCnt == 5)
		{
			if (dig_id==1)
			{
				// rain disable
				RAIN_DISABLE();	
				
				MenuDoneCnt = 0;
				sw1=0;
				sw2=0;
			}
			else if (dig_id==2)
			{
				//low flow disable
				LF_DISABLE();
				
				MenuDoneCnt = 0;
				sw1=0;
				sw2=1;
			}
			else if (dig_id==3)
			{
				HAL_IWDG_Refresh(&hiwdg); // zz7
				
				// rain enable
				uint8_t *DData;
				// eprom rain bit write
				rain_read_disable_flg=0;
				DData = &rain_read_disable_flg;
				EEPROM_WriteNBytes(EP_RAIN_DISABLE_BIT, DData, 1);
				
				
				MenuDoneCnt = 0;
				sw1=0;
				sw2=0;
				
				LCD_Clear();
				LCD_Gotoxy(0, 0);
				LCD_Puts("RAIN ENABLE");			
				LCD_Gotoxy(0, 1);
				LCD_Puts("NOW REBOOTING");
				HAL_Delay(2000);
				
				HAL_NVIC_SystemReset();
				
			}
			else if (dig_id==4)
			{
				HAL_IWDG_Refresh(&hiwdg); // zz7
				
				// LF enable
				uint8_t *DData;
				// eprom LF bit write
				lf_read_disable_flg=0;
				DData = &lf_read_disable_flg;
				EEPROM_WriteNBytes(EP_LF_DISABLE_BIT, DData, 1);
				
				
				MenuDoneCnt = 0;
				sw1=0;
				sw2=0;
				
				LCD_Clear();
				LCD_Gotoxy(0, 0);
				LCD_Puts("LF ENABLE");			
				LCD_Gotoxy(0, 1);
				LCD_Puts("NOW REBOOTING");
				HAL_Delay(2000);
				
				HAL_NVIC_SystemReset();
			}
			else if (dig_id==5)
			{
				//cancel //exit
				break;
			}
			
		}	
	}
}

void start_modem_config()
{
	HAL_IWDG_Refresh(&hiwdg); // zz7
	memset(Reply,0,1000);
	
	// test gsm process check AT OK
	LCD_Clear();
	LCD_Gotoxy(0, 0);
	LCD_Puts("STARTING");
	LCD_Gotoxy(0, 1);
	LCD_Puts("Communication");
	// TEST GSM MODEM 
	
	Head_data2 = ECBuffer - hdma_usart2_rx.Instance->CNDTR;
	testgsm();
	Tail_data2 =  ECBuffer - hdma_usart2_rx.Instance->CNDTR;
	
	HAL_Delay(500);

	// By Prashant

	memcpy(buff, &Reply[Head_data2 + 7], 2);

	if((buff[0] == 0x4F) && (buff[1] == 0x4B)){
		LCD_Clear();
		LCD_Gotoxy(0, 0);
		sprintf(msg, "4G MODULE ");
		LCD_Puts(msg);
		LCD_Gotoxy(0, 1);
		LCD_Puts("Working");
		HAL_Delay(400);
	}


	HAL_IWDG_Refresh(&hiwdg); // zz7
	
	LCD_Clear();
	LCD_Gotoxy(0, 0);
	LCD_Puts("DETECTING");
	LCD_Gotoxy(0, 1);
	LCD_Puts("SIM");
  Head_data3 = ECBuffer - hdma_usart2_rx.Instance->CNDTR;
	detect_sim();
	Tail_data3 = ECBuffer - hdma_usart2_rx.Instance->CNDTR;
	HAL_Delay(1500);

	// By Prashant 

	HAL_IWDG_Refresh(&hiwdg); // zz7
	
	memcpy(buff1, &Reply[Head_data3 + 21], 2);

	HAL_Delay(100);
	
	if(buff1[0] == 0x4F && buff1[1] == 0x4B)
	{
		no_sim_counter=0;
		
		LCD_Clear();
		LCD_Puts("SIM DETECTED");
		Sim_detected_flag = 1;
		HAL_Delay(500);
	}
	else{
	  LCD_Clear();
		LCD_Puts("SIM NOT DETECTED");
		Sim_detected_flag = 0;
		HAL_Delay(500);
	}


	LCD_Clear();
	LCD_Gotoxy(0, 0);
	LCD_Puts("READING");
	LCD_Gotoxy(0, 1);
	LCD_Puts("IMEI");
	// READ IMEI NO 
	Head_data4 = ECBuffer - hdma_usart2_rx.Instance->CNDTR;
	get_imei();	
  Tail_data4 = ECBuffer - hdma_usart2_rx.Instance->CNDTR;	
	HAL_Delay(2000);

	// By Prashant

	HAL_IWDG_Refresh(&hiwdg); // zz7
	
	if(Sim_detected_flag){

		memcpy(imei_id, &Reply[Head_data4 + 2], 15);

//		HAL_Delay(100);
//		strcat(topic_pub,imei_id);
//		HAL_Delay(100);
//		strcat(topic_sub,imei_id);
	}
	else{

		memcpy(imei_id, &Reply[Head_data4 + 2], 15);

//		strcat(topic_pub,imei_id);
	}
	HAL_Delay(100);

	LCD_Clear();
	LCD_Gotoxy(0, 0);
	sprintf(msg, "IMEI ");
	LCD_Puts(msg);
	LCD_Gotoxy(0, 1);
	LCD_Puts(imei_id);
	HAL_Delay(1000);

	hdma_usart2_rx.Instance->CNDTR=499U;
	HAL_Delay(100);


	HAL_IWDG_Refresh(&hiwdg); // zz7
	
	//******************//
	//****Sim based Functions*********//		

	if(Sim_detected_flag)
	{

		LCD_Clear();
		LCD_Gotoxy(0, 0);
		LCD_Puts("SEARCHING ");
		LCD_Gotoxy(0, 1);
		LCD_Puts("NETWORK");
		// SEARCH NETWORK
		set_nw_auto();
		HAL_Delay(2000);
		
		HAL_IWDG_Refresh(&hiwdg); // zz7
			
		network_chk(); // not boot
		
		// SETTING APN 
		LCD_Clear();
		LCD_Gotoxy(0, 0);
		LCD_Puts("SETTING APN");
		LCD_Gotoxy(0, 1);
		// Copy only first 16 characters
		memset(lcd_data, 0x00, sizeof(lcd_data));
		strncpy(lcd_data, apn, 16);
		lcd_data[16] = '\0'; // Null-terminate to prevent garbage	
		LCD_Puts(lcd_data);
		HAL_Delay(100);
		setAPN(apn);	
		HAL_Delay(1000);

		HAL_IWDG_Refresh(&hiwdg); // zz7
		
//		HAL_Delay(500); // kkk
//		signal_strengt();
//		HAL_Delay(1000);

		HAL_IWDG_Refresh(&hiwdg); // zz7

		LCD_Clear();
		LCD_Gotoxy(0, 0);
		LCD_Puts("CONNECTING TO");
		LCD_Gotoxy(0, 1);
		LCD_Puts("AUTOMAT SERVER");

		keep_alive();
		HAL_Delay(1000);
		HAL_IWDG_Refresh(&hiwdg); // zz7
		serverIP(server,port);
		HAL_Delay(4000);

		HAL_IWDG_Refresh(&hiwdg); // zz7
		if (nw_err_validity_flag==0)
		{
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			LCD_Puts("CONNECTION");
			LCD_Gotoxy(0, 1);
			LCD_Puts("DONE");
		}
		else if (nw_err_validity_flag==1)
		{
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			LCD_Puts("CONNECTION");
			LCD_Gotoxy(0, 1);
			LCD_Puts("NOT DONE");
		}
		
		set_userpwd(imei_id);

		//		mqtt_connect();
		HAL_Delay(1000);
		sub_topic(topic_sub);
		HAL_Delay(1500);
		memset(Reply,0,300);
		HAL_Delay(500);
		
		HAL_IWDG_Refresh(&hiwdg); // zz7
		
	}
	
}



//void check_sub_data_fn (char *check_sub_data, char *key_str)
void check_sub_data_fn ()
{

		HAL_IWDG_Refresh(&hiwdg); // zz7
	

    char *found = strstr(check_sub_data, "+QMTRECV:");


   
		if (found)
		{
			char *pg_data=strstr(check_sub_data, "\"v\":");
			char *flg_data=strstr(check_sub_data, "\"apn\":");
			HAL_Delay(200);
			
			if (pg_data)
			{
				//qmt_recv_count++; // ^^test
				//int index = found - hh;
				key_idx=found-check_sub_data; // uint8_t key_idx=1010-1000
				sub_receive_flag = 1;
			}
			else if (flg_data)
			{
				HAL_IWDG_Refresh(&hiwdg); // zz7
				
				char *apn_4 = strstr(check_sub_data, "\"apn\":[");
				char *rst_on = strstr(check_sub_data, "\"rst\":1");
				char *pkt_t = strstr(check_sub_data, "\"pkt\":0");
				char *rain_disable = strstr(check_sub_data, "\"rns\":1");
				char *lf_disable = strstr(check_sub_data, "\"lfs\":1");
				char *rain_enable = strstr(check_sub_data, "\"rns\":2");
				char *lf_enable = strstr(check_sub_data, "\"lfs\":2");
				char *live_pktt = strstr(check_sub_data, "\"lpk\":1");
				char *clear_water_m = strstr(check_sub_data, "\"cwm\":1");
				char *pit_set = strstr(check_sub_data, "\"pit\":0");
			
				
				if (pit_set==NULL)					
				{
					HAL_IWDG_Refresh(&hiwdg); // zz7
					uint8_t *Data;
					
					pit_val_chk=one_digit_parser_2nd_config ("\"pit\":");
					
					if (pit_val_chk!=0)
					{
						pumpInitiationTimeSecs=pit_val_chk;

						Data = &pumpInitiationTimeSecs;
						EEPROM_WriteNBytes(EP_PUMPINITSECS, Data, 1); // Error Flag
						
						LCD_Clear();
						LCD_Gotoxy(0, 0);
						LCD_Puts("PIT SET");			
						LCD_Gotoxy(0, 1);
						LCD_Puts("NOW REBOOTING");
						
						HAL_Delay(500);
						pit_msg_confirmed(); // confirmation msg
						HAL_Delay(1000);

						HAL_NVIC_SystemReset();  // @@ ##	
					}
				}
				
				if (live_pktt)
				{
					if (lf_read_disable_flg!=1)
					{
						low_flow_status1 = HAL_GPIO_ReadPin(LOW_FLOW_GPIO_Port, LOW_FLOW_Pin); 
						low_flow_status = low_flow_status1;
					}
					
					HAL_Delay(500);
					pub_live_msg();
				}
				
				if (apn_4)
				{
					//iot sim apn set by mqtt
					HAL_IWDG_Refresh(&hiwdg); // zz7
					memset(extracted_buffer5,0,sizeof(extracted_buffer5));
					memcpy(extracted_buffer5,&Reply[CNDTR_Buff_check_previous],300);
					
					bt_apn_parser ("\"apn\":[");
					APN_SET_REMOTE(apn);
				}
				
				if (pkt_t==NULL)					
				{
					HAL_IWDG_Refresh(&hiwdg); // zz7
					uint8_t *Data;
					uint8_t *DData;
					
					custom_pkt_time_val_chk=one_digit_parser_2nd_config ("\"pkt\":");
					
					if (custom_pkt_time_val_chk!=0)
					{
						custom_pkt_time=custom_pkt_time_val_chk;
						
						Data = (uint8_t *)&custom_pkt_time;
						EEPROM_WriteNBytes(EP_CUSTOM_PKT_TIME, Data, 2); // Error Flag
						
						// pkt_t_store_bit write
						pkt_t_store_bit=1;
						DData = &pkt_t_store_bit;
						EEPROM_WriteNBytes(EP_pkt_t_STORE_BIT, DData, 1);
						
						HAL_Delay(100);
						pkt_t_msg_confirmed(); // confirmation msg
						HAL_Delay(1000);

						HAL_NVIC_SystemReset();  // @@ ##	
					}
					
				}
				
				
				if (rst_on)
				{
					HAL_IWDG_Refresh(&hiwdg); // zz7
					
					HAL_Delay(500);
					LCD_Clear();
					LCD_Gotoxy(0, 0);
					LCD_Puts("REBOOTING");			

					HAL_Delay(500);
					rst_msg_confirmed();
					HAL_Delay(1500);
					
					HAL_NVIC_SystemReset();
				}
				
				//=================
				if (rain_disable)
				{
					// rain disable
					HAL_IWDG_Refresh(&hiwdg); // zz7
			
					// eprom rain bit write
					HAL_Delay(500);
					
					uint8_t *DData;
					
					rain_read_disable_flg=1;
					DData = &rain_read_disable_flg;
					EEPROM_WriteNBytes(EP_RAIN_DISABLE_BIT, DData, 1);
					
					LCD_Clear();
					LCD_Gotoxy(0, 0);
					LCD_Puts("RAIN DISABLE");			
					LCD_Gotoxy(0, 1);
					LCD_Puts("NOW REBOOTING");
					HAL_Delay(500);
					rns_msg_confirmed();
					HAL_Delay(1500);
					
					HAL_NVIC_SystemReset();
				}
				
				if (lf_disable)
				{
					//low flow disable
					HAL_IWDG_Refresh(&hiwdg); // zz7
			
					// eprom LF bit write
					HAL_Delay(500);
					
					uint8_t *DData;
					
					lf_read_disable_flg=1;
					DData = &lf_read_disable_flg;
					EEPROM_WriteNBytes(EP_LF_DISABLE_BIT, DData, 1);
					
					LCD_Clear();
					LCD_Gotoxy(0, 0);
					LCD_Puts("LF DISABLE");			
					LCD_Gotoxy(0, 1);
					LCD_Puts("NOW REBOOTING");
					HAL_Delay(500);
					lfs_msg_confirmed();
					HAL_Delay(1500);
					
					HAL_NVIC_SystemReset();
				}
				
				if (rain_enable)
				{
					HAL_IWDG_Refresh(&hiwdg); // zz7
				
					// rain enable
					uint8_t *DData;
					// eprom rain bit write
					rain_read_disable_flg=0;
					DData = &rain_read_disable_flg;
					EEPROM_WriteNBytes(EP_RAIN_DISABLE_BIT, DData, 1);
					
									
					LCD_Clear();
					LCD_Gotoxy(0, 0);
					LCD_Puts("RAIN ENABLE");			
					LCD_Gotoxy(0, 1);
					LCD_Puts("NOW REBOOTING");
					HAL_Delay(500);
					rns_msg_confirmed();
					HAL_Delay(1500);
					
					HAL_NVIC_SystemReset();	
				}
				
				if (lf_enable)
				{
					HAL_IWDG_Refresh(&hiwdg); // zz7
				
					// LF enable
					uint8_t *DData;
					// eprom LF bit write
					lf_read_disable_flg=0;
					DData = &lf_read_disable_flg;
					EEPROM_WriteNBytes(EP_LF_DISABLE_BIT, DData, 1);
					
					LCD_Clear();
					LCD_Gotoxy(0, 0);
					LCD_Puts("LF ENABLE");			
					LCD_Gotoxy(0, 1);
					LCD_Puts("NOW REBOOTING");
					HAL_Delay(500);
					lfs_msg_confirmed();
					HAL_Delay(1500);
					
					HAL_NVIC_SystemReset();
				}
				
				if (clear_water_m)
				{
					//water-flow meter reset
					HAL_Delay(500);
					uint8_t *Datta;
				
					Total_Flow1_count=0;
					Datta = (uint8_t *)&Total_Flow1_count;
					EEPROM_WriteNBytes(EP_WATER_M, Datta, 4);
					
					LCD_Clear();
					LCD_Gotoxy(0, 0);
					LCD_Puts("FLOW METER RESET");			
					LCD_Gotoxy(0, 1);
					LCD_Puts("DONE, REBOOTING");
					HAL_Delay(500);
					cwm_msg_confirmed();
					HAL_Delay(1500);
					
					HAL_NVIC_SystemReset();
				}
				//=======================
				

			}
		}
}


void j_str_validity_fn (char *key_str)
{

    // locate the key_str in check_sub_data
    char *chk_1 = strstr(j_str, key_str);
		char *chk_2 = strstr(j_str, key_str);
    //printf("y %s\n", start); // give null if key_str is not present 
   
		if (chk_1)
		{
			j_str_validity_flag = 1;
		}
		
		if (chk_2)
		{
			j_str_validity_flag_2 = 1;
		}
}



void stat_error_chk_fn ()
{
    // locate the key_str in check_sub_data
    char *chkk = strstr(stat_err_buff, "+QMTSTAT:");
		
		if (chkk)
		{
			stat_error_chk_flag = 1;
		}
		else 
		{
			char *chkk_b = strstr(stat_err_buff, "ERROR\r\n\r\nERROR");
			
			if (chkk_b)
			{
				stat_error_chk_flag = 2;
			}
		}
}



void APN_IP_n_PASS_READ()
{
	uint8_t idx;
	uint8_t *Data;
	uint8_t *Data_ip_bit;
	uint8_t *Data_apn_bit;
	uint8_t *Data_ser;
	uint8_t *Data_apn;
	
	//=====sensor read==================
	uint8_t *Data_rain_bit;
	uint8_t *Data_lf_bit;	
	
	Data_rain_bit=&rain_read_disable_flg;
	EEPROM_ReadNBytes(EP_RAIN_DISABLE_BIT, Data_rain_bit, 1);

	if (rain_read_disable_flg==1)
	{
		rain_status=0;
		sensor_status=1;
		
		LCD_Clear();
		LCD_Gotoxy(0, 0);
		LCD_Puts("RAIN SENSOR");
		LCD_Gotoxy(0, 1);
		LCD_Puts("DISABLE");
		HAL_Delay(500);
	}
	
	Data_lf_bit=&lf_read_disable_flg;
	EEPROM_ReadNBytes(EP_LF_DISABLE_BIT, Data_lf_bit, 1);
	
	if (lf_read_disable_flg==1)
	{
		low_flow_status=0;
		sensor_status=2;
		
		LCD_Clear();
		LCD_Gotoxy(0, 0);
		LCD_Puts("LF SENSOR");
		LCD_Gotoxy(0, 1);
		LCD_Puts("DISABLE");
		HAL_Delay(200);
	}
	
	if(rain_read_disable_flg==1 && lf_read_disable_flg==1)
	{
		sensor_status=3;
	}
	//===== sensor end============================ 

	// EP_IP_STORE bit read
	Data_ip_bit = &IP_store_bit;
	EEPROM_ReadNBytes(EP_IP_STORE, Data_ip_bit, 1); 
	
	// if IP_store_bit set then read EP_server_IP otherwise use default IP 
	if (IP_store_bit != 1)  
	{
		//IP_store_bit=1; // temp, comment it
		// default ip
		//strncpy(server, "15.206.24.163", sizeof(server) - 1);
		strncpy(server, "103.93.94.103", sizeof(server) - 1);
    server[sizeof(server) - 1] = '\0'; // for null char
		
//			LCD_Clear();
//			LCD_Gotoxy(0, 0);
//			LCD_Puts("PLEASE SET IP");
//			LCD_Gotoxy(0, 1);
//			LCD_Puts("FROM MENU");
//			HAL_Delay(1000);
	}
	else
	{
		// serverIP read for eprom
		Data_ser = (uint8_t *)&server[0];
		for (idx = 0; idx < 16; idx++)
		{
			Data_ser[idx] = EEPROM_ReadByte(EP_server_IP + idx);
		}
	}

	HAL_IWDG_Refresh(&hiwdg); // zz7
	
	Data_apn_bit=&APN_store_bit;
	EEPROM_ReadNBytes(EP_APN_STORE, Data_apn_bit, 1);
	
	// if APN_store_bit set then read EP_apn otherwise use default APN 
	if (APN_store_bit != 1)  
	{
		//APN_store_bit=1; // temp, comment it
		//default apn
		//APN_SET_FN("jionet");
		APN_SET_FN("airtelgprs.com");
		
//			LCD_Clear();
//			LCD_Gotoxy(0, 0);
//			LCD_Puts("PLEASE SET APN");
//			LCD_Gotoxy(0, 1);
//			LCD_Puts("FROM MENU");
//			HAL_Delay(1000);
	}
	else
	{
		// apn read from eprom
		Data_apn = (uint8_t *)&apn[0];
		for (idx = 0; idx < 65; idx++)
		{
			Data_apn[idx] = EEPROM_ReadByte(EP_apn + idx); 
		}
	}
	
	// =======
	// PASS_CODE bit read
	Data = &pass_bit_store;
	EEPROM_ReadNBytes(EP_PASS_STORE, Data, 1); 

	// PASS_CODE read
	if (pass_bit_store==1)
	{
		Data = (uint8_t *)&pass_ev[0];
		for (idx = 0; idx < 4; idx++)
		{
			Data[idx] = EEPROM_ReadByte(EP_PASS_CODE + idx); // Error Flag
		}
	}
	// ========
	
	HAL_IWDG_Refresh(&hiwdg); // zz7
	//========
	// PORT bit read
	Data = &PORT_store_bit;
	EEPROM_ReadNBytes(EP_PORT_STORE, Data, 1);

	// PORT NUM read
	if (PORT_store_bit==1)
	{
		Data = (uint8_t *)&port[0];
		for (idx = 0; idx < 6; idx++)
		{
			Data[idx] = EEPROM_ReadByte(EP_PORT_NUM + idx); // Error Flag
		}
	}
	
	
	//=== custom pkt time
	uint8_t *Data_pkt_bit;
	Data_pkt_bit=&pkt_t_store_bit;
	EEPROM_ReadNBytes(EP_pkt_t_STORE_BIT, Data_pkt_bit, 1);
	
	if (pkt_t_store_bit==1)
	{
		uint8_t *Data_pkt_t;
		Data_pkt_t = (uint8_t *)&custom_pkt_time;
		EEPROM_ReadNBytes(EP_CUSTOM_PKT_TIME, Data_pkt_t, 2); 
	}
	
	if (custom_pkt_time>=240)
	{
		custom_pkt_time=3;
	}
	
	
	//=== water meter eprom read===
	HAL_IWDG_Refresh(&hiwdg); // zz7
	
	Data = (uint8_t *)&Total_Flow1_count;
	EEPROM_ReadNBytes(EP_WATER_M, Data, 4);
	
	if (Total_Flow1_count==4294967295)
	{
		Total_Flow1_count=0;
	}
	
	PRE_Total_Flow1_count=Total_Flow1_count;
	//========
	
	// pit read
	Data = &pumpInitiationTimeSecs;
	EEPROM_ReadNBytes(EP_PUMPINITSECS, Data, 1); // Error Flag
	
	if (pumpInitiationTimeSecs > 90)
	{
		pumpInitiationTimeSecs = 5;
	}
	
	// flow pulse read
	Data=&TotalFlow1Counter;
	EEPROM_ReadNBytes(EP_FLOW_PULSE, Data, 1);
	if (TotalFlow1Counter > 11)
	{
		TotalFlow1Counter = 0;
	}
	
}



void net_test()
{
	if (t_var==1)
	{
		char chhkk[15] = "AT+QNWINFO\r\n"; // ++== del
		HAL_UART_Transmit_DMA (&huart2, (uint8_t *)chhkk, strlen(chhkk));
		HAL_Delay(200);
		t_var=0;
	}
}


void nw_err_validity_boot_fn (char *key_str)
{

    // locate the key_str in check_sub_data
    char *apn_chk = strstr(nw_err_buff, key_str);
   
		//if (apn_chk && nw_err_buff[31]=='O' && nw_err_buff[32]=='K')
		if (apn_chk)
		{
			nw_err_validity_boot_flag = 1;
		}

}

void network_boot_chk()
{
		memset(nw_err_buff, 0x00, sizeof(nw_err_buff));
		HAL_Delay(500);
	
		Head = ECBuffer - hdma_usart2_rx.Instance->CNDTR;
		//char chkk[15] = "AT+COPS?\r\n"; // ++== del
		char chkk[15] = "AT+QNWINFO\r\n"; // ++== del
		HAL_UART_Transmit_DMA (&huart2, (uint8_t *)chkk, strlen(chkk));
		HAL_Delay(800);
		Tail = hdma_usart2_rx.Instance->CNDTR;
		Tail = ECBuffer - Tail;
		HAL_Delay(30);
		memcpy(nw_err_buff, &Reply[Head],(Tail-Head));
		nw_err_validity_boot_fn ("No Service");
		
		//at boot
		if (nw_err_validity_boot_flag==1)
		{
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			LCD_Puts("MOBILE NETWORK");
			LCD_Gotoxy(0, 1);
			LCD_Puts("NOT FOUND");
			HAL_Delay(3000);
		}
		else if (nw_err_validity_boot_flag==0)
		{
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			LCD_Puts("MOBILE NETWORK");
			LCD_Gotoxy(0, 1);
			LCD_Puts("FOUND");
			HAL_Delay(1500);
		}
		
		Head=0;
		Tail=0;
}

// ======== not boot===========
void nw_err_validity_fn (char *key_str)
{

    // locate the key_str in check_sub_data
    char *apn_chk = strstr(nw_err_buff, key_str);
   
		//if (apn_chk && nw_err_buff[31]=='O' && nw_err_buff[32]=='K')
		if (apn_chk)
		{
			nw_err_validity_flag = 1;
		}
		else
		{
			nw_err_validity_flag=0;
		}
}

void network_chk()
{
		HAL_IWDG_Refresh(&hiwdg); // zz7
	
		memset(nw_err_buff, 0x00, sizeof(nw_err_buff));
		HAL_Delay(500);
	
		Head = ECBuffer - hdma_usart2_rx.Instance->CNDTR;
		char chkk[15] = "AT+QNWINFO\r\n"; 
		HAL_UART_Transmit_DMA (&huart2, (uint8_t *)chkk, strlen(chkk));
		HAL_Delay(800);
		Tail = hdma_usart2_rx.Instance->CNDTR;
		Tail = ECBuffer - Tail;
		HAL_Delay(30);
		memcpy(nw_err_buff, &Reply[Head],(Tail-Head));
		nw_err_validity_fn ("No Service");
		
		// not in boot
		if (Sim_detected_flag==0)
		{
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			LCD_Puts("SIM NOT");
			LCD_Gotoxy(0, 1);
			LCD_Puts("DETECTED");
			HAL_Delay(3000);
		}
		else if (nw_err_validity_flag==1)
		{
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			LCD_Puts("MOBILE NETWORK");
			LCD_Gotoxy(0, 1);
			LCD_Puts("NOT FOUND");
			HAL_Delay(3000);
		}
		else if (nw_err_validity_flag==0)
		{
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			LCD_Puts("MOBILE NETWORK");
			LCD_Gotoxy(0, 1);
			LCD_Puts("FOUND");
			HAL_Delay(1500);
		}
		
		Head=0;
		Tail=0;
}



void chk_cloud_conn()
{
	memset(buff_Check_conn, 0x00, sizeof(buff_Check_conn)); 
	HAL_Delay(500);
	
	Head_data = ECBuffer - hdma_usart2_rx.Instance->CNDTR;
	check_mqtt_status();
	Tail_data = ECBuffer - hdma_usart2_rx.Instance->CNDTR;
	
	//HAL_Delay(10); //ori
	HAL_Delay(200); // 25n

	memcpy(buff_Check_conn, &Reply[Head_data],(Tail_data-Head_data));

	if(buff_Check_conn[14]=='3')
	{
		HAL_IWDG_Refresh(&hiwdg); // zz7
		
		LCD_Clear();
		LCD_Gotoxy(0, 0);
		LCD_Puts("CLOUD CONN");
		LCD_Gotoxy(0, 1);
		LCD_Puts("CONNECTED");
		HAL_Delay(4000);
	}
	else
	{
		HAL_IWDG_Refresh(&hiwdg); // zz7
		
		LCD_Clear();
		LCD_Gotoxy(0, 0);
		LCD_Puts("CLOUD CONN");
		LCD_Gotoxy(0, 1);
		LCD_Puts("NOT CONNECTED");
		HAL_Delay(4000);
	}					
}

void sig_strength()
{
	HAL_IWDG_Refresh(&hiwdg); // zz7
	
	Head_data1 = ECBuffer - hdma_usart2_rx.Instance->CNDTR;
	signal_strengt();
	Tail_data1 = ECBuffer - hdma_usart2_rx.Instance->CNDTR;
	HAL_Delay(10);
	
	memcpy(buff_signal_strength, &Reply[Head_data1],12);
	if(buff_signal_strength[3]== 'C' && buff_signal_strength[4]== 'S' && buff_signal_strength[5]== 'Q' )
	{
		memcpy(buff_signal_strength_data, &buff_signal_strength[8],2);
		uint8_t signal_value = atoi(buff_signal_strength_data);
		if(signal_value >= 16 && signal_value <= 40)
		{			
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			LCD_Puts("NETWORK STRENGTH");
			LCD_Gotoxy(0, 1);
			LCD_Puts("GOOD");
			HAL_Delay(1000);
		}
		else if(signal_value < 16 && signal_value >= 8)
		{
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			LCD_Puts("NETWORK STRENGTH");
			LCD_Gotoxy(0, 1);
			LCD_Puts("MEDIUM");
			HAL_Delay(1000);
		}
		else{
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			LCD_Puts("NETWORK STRENGTH");
			LCD_Gotoxy(0, 1);
			LCD_Puts("BAD");
			HAL_Delay(1000);
		}
	}
}





//diagnostic function
void dig()
{	
	MenuDoneCnt = 0;
	
	LCD_Clear();
	sw1=0;
	sw2=0;
	backflag=0;
	
	uint8_t syss_i[5] = {1, 2, 3, 4, 5}; // ##
	char syss_c[5][16] = {"1. MOBILE NW", "2. CLOUD CONN", "3. MODEM RESTART", "4. SIG STRENGTH" ,"5. EXIT"}; // ##
	while (1)
	{
		HAL_IWDG_Refresh(&hiwdg); // zz7
		
		LCD_Clear();
		dig_id = syss_i[sw2];
		sw2_max = 5;
		LCD_Gotoxy(0, 0);
		LCD_Puts("DIAGNOSTIC CHK:");
		LCD_Gotoxy(0, 1);
		sprintf(msg, "%s     ", syss_c[sw2]);
		LCD_Puts(msg);		
		HAL_Delay(500);

		if ((sw1 == 1) && (input_timeout < 1000))
		{
			MenuDoneCnt = 5;
			sw1=0;
			sw2=0;
		}
		
		if (MenuDoneCnt == 5)
		{
			if (dig_id==1)
			{
				// CHK MOBILE NW
				network_chk(); // not boot fn
				//break; //ori
				
				MenuDoneCnt = 0;
				sw1=0;
				sw2=0;
			}
			else if (dig_id==2)
			{
				//CHK CLOUD CONN
				chk_cloud_conn();
				//break;
				
				MenuDoneCnt = 0;
				sw1=0;
				sw2=1;
			}
			else if (dig_id==3)
			{
				HAL_IWDG_Refresh(&hiwdg); // zz7
				
				//modem restart
				LCD_Clear();
				LCD_Gotoxy(0, 0);
				LCD_Puts("MODEM RESTART");
				LCD_Gotoxy(0, 1);
				LCD_Puts("RECONNECTING");
				HAL_Delay(1000);
				
				HAL_IWDG_Refresh(&hiwdg); // zz7
				stop_modem();
				HAL_Delay(2000);
				
				HAL_IWDG_Refresh(&hiwdg); // zz7
				
				start_modem();
				HAL_Delay(2000);
				
				HAL_IWDG_Refresh(&hiwdg); // zz7
				
				HAL_UART_DeInit(&huart2);
				HAL_UART_Init(&huart2);
				HAL_UART_Receive_DMA (&huart2, (uint8_t *)Reply,ECBuffer);
				
				HAL_IWDG_Refresh(&hiwdg); // zz7
				
				HAL_Delay(2000);
				start_modem_config();
				
				HAL_IWDG_Refresh(&hiwdg); // zz7
				
				HAL_Delay(1000);
				////&&**live_values(); // to remove stuck char
				pub_live_msg();
				//break;
				
				MenuDoneCnt = 0;
				sw1=0;
				sw2=2;		
			}
			else if (dig_id==4)
			{
				//sig strenght
				//break;
				sig_strength();
				
				MenuDoneCnt = 0;
				sw1=0;
				sw2=3;
			}
			else if (dig_id==5)
			{
				//cancel //exit
				break;
			}
			
		}	
	}
}

void stop_all_valve()
{
		// off
		HAL_GPIO_WritePin(RELAY_PWR_GPIO_Port,RELAY_PWR_Pin,1);
		HAL_GPIO_WritePin(PUMP_R_GPIO_Port,PUMP_R_Pin,0);
	
		memset(val_PB, 0, sizeof(val_PB));
		memset(val_PA, 0, sizeof(val_PA));
	
		val_M=0;
		vv_[2]=0;vv_[3]=0;vv_[4]=0;vv_[5]=0;vv_[6]=0;vv_[7]=0;vv_[8]=0;vv_[9]=0;vv_[10]=0;vv_[11]=0;vv_[12]=0;vv_[13]=0;vv_[14]=0;vv_[15]=0;
	
		mcp_return=MCP_PB_WRITE(0x00);
		mcp_return=MCP_PA_WRITE(0x00);
	
		Run_State=0;
}



void Time_at_SIM()
{
			HAL_IWDG_Refresh(&hiwdg); // zz7
		
//			LCD_Clear();
//			LCD_Gotoxy(0, 0);
//			LCD_Puts("GETTING ");
//			LCD_Gotoxy(0, 1);
//			LCD_Puts("NETWORK TIME");
			

			HAL_Delay(1000);

			Head = ECBuffer - hdma_usart2_rx.Instance->CNDTR;
			
		
			get_time_sim();
			HAL_Delay(100); // old 0 ==>2.6
			Tail = hdma_usart2_rx.Instance->CNDTR;
			Tail = ECBuffer - Tail;
			HAL_Delay(10);
	//		for(int i =Head ; i<=Tail;i++){
				memcpy(buff_time, &Reply[Head],(Tail-Head));
	//		}

			if(buff_time[39]!= 'O' && buff_time[40]!= 'K'){
			memset(buff_time, 0x00, sizeof(buff_time));
			HAL_Delay(600); // old 500 ==>2.6

			Head = ECBuffer - hdma_usart2_rx.Instance->CNDTR;
			get_time_sim();
			HAL_Delay(800); // old 500 ==>2.6
			Tail = hdma_usart2_rx.Instance->CNDTR;
			Tail = ECBuffer - Tail;
			HAL_Delay(30); // old 10 ==>2.6
			memcpy(buff_time, &Reply[Head],(Tail-Head));
			}
			
			HAL_IWDG_Refresh(&hiwdg); // zz7

			//if(buff_time[0] == 0x0D && buff_time[1] == 0x0A){ //ori
			if(buff_time[39] == 'O' && buff_time[40] == 'K'){
				memcpy(buff_date, &buff_time[10],10); //21
					
				memcpy(buff_date_yy,&buff_date[0],4);
				memcpy(buff_date_mm,&buff_date[5],2);
				memcpy(buff_date_dd,&buff_date[8],2);
					
					
				memcpy(buff_time1, &buff_time[21],5);
					
				memcpy(buff_time1_hrs,&buff_time1[0],2);
				memcpy(buff_time1_min,&buff_time1[3],2);		
				
			 
				
				LCD_Clear();
				LCD_Puts("DATE ");
				LCD_Puts(buff_date);
				HAL_Delay(100);
				LCD_Gotoxy(0, 1);
				LCD_Puts("TIME ");
				LCD_Puts(buff_time1);
				HAL_Delay(100);
				time_ok=1; //get correct time from network			
		}

		HAL_IWDG_Refresh(&hiwdg); // zz7
		
		HAL_Delay(100);
		
	// sim_new	
	if (time_ok==1)
	{
		HAL_IWDG_Refresh(&hiwdg); // zz7
		
		get_year = atoi(buff_date_yy);
		get_month = atoi(buff_date_mm);
		get_date =  atoi(buff_date_dd);
		
		
		get_hrs = atoi(buff_time1_hrs);
		get_mins = atoi(buff_time1_min);
		get_year=(get_year-2000);
		HAL_Delay(100);
		
		k_t =get_date,m_t = get_month,C_data =20,D_data = get_year;
		
				// Adjusting for months January and February
			if (m_t == 1 || m_t == 2) {
					m_t += 12;
					D_data -= 1;
			}
			
		
			k_cen = D_data % 100;  // Year within century
			j_cen = D_data / 100;  // Century
			
			F_data = (k_t+ (13 * (m_t + 1)) / 5 + k_cen + (k_cen / 4) + (j_cen / 4) - (2 * j_cen)) % 7;
		
			//(h + 7) % 7;
			
			F_data=(F_data + 7) % 7;
			
			
			if (F_data==0)
			{
				F_data=6;
			}
			else if (F_data==1)
			{
				F_data=7;
			}
			else
			{
				F_data=F_data-1;
			}
			
			get_day=F_data;

		MX_RTC_Init();
		HAL_Delay(500);
		RTC_TimeShow();
		HAL_Delay(1000);
	}
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
	uint8_t idx, j, i, u; // ori
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
	
	HAL_GPIO_WritePin(EXP_OUT_GPIO_Port,EXP_OUT_Pin,1);
	
  MX_DMA_Init();
  MX_I2C2_Init();
  
	HAL_Delay(1000);
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
	MX_IWDG_Init(); // zz7
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(RELAY_PWR_GPIO_Port,RELAY_PWR_Pin,1);
////	HAL_UART_Receive_DMA (&huart2, (uint8_t *)Reply,ECBuffer);
////  modem_state control pin to set in reset state
////	HAL_GPIO_WritePin(PWR_MODEM_GPIO_Port,PWR_MODEM_Pin,1);
////	
////	testgsm();
	
	HAL_IWDG_Refresh(&hiwdg); // zz7 // just after MX_IWDG_Init();


	EEPROM_Init(AT24C16);
	
	LCD_Init();
	lcd_on();
//	//HAL_GPIO_WritePin(LCD_LED_GPIO_Port, LCD_LED_Pin, 1);
	HAL_Delay(500);
	
	
	LCD_Gotoxy(0, 0);
	LCD_Puts("   AUTOMAT      ");
	LCD_Gotoxy(0, 1);
	LCD_Puts("  INDUSTRIES   ");
	HAL_Delay(1000);
	LCD_Clear();
	LCD_Gotoxy(0, 0);
	LCD_Puts(" AUTODRIP SMART");
	LCD_Gotoxy(0, 1);
	LCD_Puts("  CONTROLLER    ");
	
	HAL_IWDG_Refresh(&hiwdg); // zz7
	
	HAL_Delay(700);
	APN_IP_n_PASS_READ(); // from eprom
	
	HAL_IWDG_Refresh(&hiwdg); // zz7
	
	// Start Modem ec 200 

	start_modem();
	HAL_Delay(2000);
	HAL_TIM_Base_Start_IT(&htim14);
	HAL_TIM_Base_Start_IT(&htim6);
	
	HAL_IWDG_Refresh(&hiwdg); // zz7
	
	LCD_Clear();
	LCD_Gotoxy(0, 0);
	LCD_Puts("START FW 1.0");
	// HAL_Delay(500); //kkk
	LCD_Gotoxy(0, 1);
	LCD_Puts("READING CONFIG");
	
	HAL_IWDG_Refresh(&hiwdg); // zz7
	
	HAL_Delay(1000);
	RTC_TimeShow();
	index1 = 1;
		
	HAL_IWDG_Refresh(&hiwdg); // zz7
	
		// test gsm process check AT OK
		LCD_Clear();
		LCD_Gotoxy(0, 0);
		LCD_Puts("STARTING");
		LCD_Gotoxy(0, 1);
		LCD_Puts("COMMUNICATION");
		// TEST GSM MODEM 
		testgsm();
		HAL_Delay(500);

			// By Prashant
//	for(int i = 7; i<=8;i++){
		
		memcpy(buff, &Reply[8], 2);
//	}
	

	if(buff[0] == 0x4F && buff[1] == 0x4B){
		LCD_Clear();
		LCD_Gotoxy(0, 0);
		sprintf(msg, "4G MODULE ");
		LCD_Puts(msg);
		LCD_Gotoxy(0, 1);
		LCD_Puts("WORKING");
		HAL_Delay(400);
	}
		
	HAL_IWDG_Refresh(&hiwdg); // zz7
	

		LCD_Clear();
		LCD_Gotoxy(0, 0);
		LCD_Puts("DETECTING");
		LCD_Gotoxy(0, 1);
		LCD_Puts("SIM");
		
		Head_data5 = ECBuffer - hdma_usart2_rx.Instance->CNDTR;
		detect_sim();
		Tail_data5 = ECBuffer - hdma_usart2_rx.Instance->CNDTR;
		HAL_Delay(1000);
		
	// By Prashant 
		HAL_IWDG_Refresh(&hiwdg); // zz7
		
//	for(int i = 32; i<=33;i++){
		memcpy(buff1, &Reply[Head_data5+21], 2*sizeof(char));
//	}
//		

	HAL_Delay(100);

	if(buff1[0] == 0x4F && buff1[1] == 0x4B)
	{
		LCD_Clear();
		LCD_Puts("SIM DETECTED");
		Sim_detected_flag = 1;
		HAL_Delay(500);
	}
	else{
	  LCD_Clear();
		LCD_Puts("SIM NOT DETECTED");
		Sim_detected_flag = 0;
		HAL_Delay(500);
	}
	
	
		
	HAL_IWDG_Refresh(&hiwdg); // zz7
	

		LCD_Clear();
		LCD_Gotoxy(0, 0);
		LCD_Puts("READING");
		LCD_Gotoxy(0, 1);
		LCD_Puts("IMEI");
		// READ IMEI NO 
		
		Head_data6 = ECBuffer - hdma_usart2_rx.Instance->CNDTR;
		get_imei();	
		Tail_data6 =ECBuffer -  hdma_usart2_rx.Instance->CNDTR;
			
		HAL_Delay(1000);
		
					// By Prashant

//		if(Sim_detected_flag){
////		for(int i = 23; i<=39;i++){
//		memcpy(imei_id, &Reply[39], 15);
////	}
//		HAL_Delay(100);
//		strcat(topic_pub,imei_id);
//	HAL_Delay(100);
//	  strcat(topic_sub,imei_id);
//}
//		else{

		memcpy(imei_id, &Reply[Head_data6+2], 15);

  	strcat(topic_pub,imei_id);
//}
		HAL_Delay(100);
		strcat(topic_sub,imei_id);
		
		HAL_IWDG_Refresh(&hiwdg); // zz7
	

	
	
	LCD_Clear();
	LCD_Gotoxy(0, 0);
	sprintf(msg, "IMEI ");
	LCD_Puts(msg);
	LCD_Gotoxy(0, 1);
	LCD_Puts(imei_id);
	HAL_Delay(1000);
	

		
		hdma_usart2_rx.Instance->CNDTR=499U;
		HAL_Delay(100);
//		HAL_UART_DMAStop(&huart2);
//		hdma_usart2_rx.Instance->CNDTR=499U;
		
		
//******************************************************//
//***********Sim based Functions************************//		
		
if(Sim_detected_flag)
		{
		
		HAL_IWDG_Refresh(&hiwdg); // zz7
		
		LCD_Clear();
		LCD_Gotoxy(0, 0);
		LCD_Puts("SEARCHING ");
		LCD_Gotoxy(0, 1);
		LCD_Puts("NETWORK");
		// SEARCH NETWORK
		set_nw_auto(); //ori
		HAL_Delay(2000);
			
		network_boot_chk();
		
		HAL_IWDG_Refresh(&hiwdg); // zz7
			
	
		// SETTING APN 
		LCD_Clear();
		LCD_Gotoxy(0, 0);
		LCD_Puts("SETTING APN");
		LCD_Gotoxy(0, 1);
		
		// Copy only first 16 characters
		memset(lcd_data, 0x00, sizeof(lcd_data));
		strncpy(lcd_data, apn, 16);
		lcd_data[16] = '\0'; // Null-terminate to prevent garbage
		
		LCD_Puts(lcd_data);
		HAL_Delay(100);
		setAPN(apn);	
		HAL_Delay(1000);



	HAL_IWDG_Refresh(&hiwdg); // zz7
		
	// getting time	
	LCD_Clear();
	LCD_Gotoxy(0, 0);
	LCD_Puts("GETTING ");
	LCD_Gotoxy(0, 1);
	LCD_Puts("NETWORK TIME");
	
	Time_at_SIM();

	HAL_IWDG_Refresh(&hiwdg); // zz7
	
		LCD_Clear();
		LCD_Puts("CONNECTING TO");
		LCD_Gotoxy(0, 1);
		LCD_Puts("AUTOMAT SERVER");
	
	HAL_IWDG_Refresh(&hiwdg); // zz7
	
		keep_alive();
		HAL_Delay(1000);
		serverIP(server,port);
		HAL_Delay(4000); //jio

	HAL_IWDG_Refresh(&hiwdg); // zz7

		if (nw_err_validity_boot_flag==0)
		{
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			LCD_Puts("CONNECTION");
			LCD_Gotoxy(0, 1);
			LCD_Puts("DONE");
		}
		else if (nw_err_validity_boot_flag==1)
		{
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			LCD_Puts("CONNECTION");
			LCD_Gotoxy(0, 1);
			LCD_Puts("NOT DONE");
		}

		set_userpwd(imei_id);
		HAL_Delay(1500);

	 memset(Reply,0,700);
   sub_topic(topic_sub);
	 HAL_Delay(1500);
	}

	HAL_IWDG_Refresh(&hiwdg); // zz7
	//Mqqt_CHK_AT_BOOT();
	pub_boot_msg();
	HAL_Delay(300);
	pub_live_msg();
	HAL_IWDG_Refresh(&hiwdg); // zz7	
	

	HAL_UART_DeInit(&huart2);
	HAL_UART_Init(&huart2);
	HAL_UART_Receive_DMA (&huart2, (uint8_t *)Reply,ECBuffer);
	HAL_Delay(200);
	
	mcp_init();
	
	// yy7
	if (mcp_mode_return==1)
	{
		HAL_IWDG_Refresh(&hiwdg); // zz7	
		
		LCD_Clear();
		LCD_Gotoxy(0, 0);
		LCD_Puts("MCP23017 MODE");
		LCD_Gotoxy(0, 1);
		LCD_Puts("ERROR");
		HAL_Delay(4000);
	}
	
	//======= reset 1min timer
	__HAL_TIM_SET_COUNTER(&htim14, 0);
	//=======
	
	currRTCtimeMins=currRTCtimeMins+1;
	

	HAL_IWDG_Refresh(&hiwdg); // zz7	
	// ^^
	while (1)
	{
		/* USER CODE END WHILE */
		
		
		if (mcp_mode_return==0 && mcp_return==0)
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
		}
		else if ((mcp_mode_return==1) || (mcp_return==1))
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			if (mcp_error_flg==0)
			{

				stop_all_valve();
				Run_State = 0;
				halt_alloperation=1;
				mcp_error_flg=1;
			}
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			LCD_Puts("MCP23017");
			LCD_Gotoxy(0, 1);
			LCD_Puts("ERROR");
			HAL_Delay(1250);
		}
		
		//===== rain- alert live_pkt
		if ((rain_status==1) && (mains_status==1) && (rain_alert_flg==0))
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			HAL_Delay(500);
			if (Sim_detected_flag==1)
			{
				//live_values();
				pub_live_msg();
			}

			rain_alert_flg=1;
		}
		else if ((rain_alert_flg==1) && (rain_status==0) && (mains_status==1))
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			if (Sim_detected_flag==1)
			{
				HAL_Delay(500);
				pub_live_msg();
			}
			rain_alert_flg=0;
		}
		
		//======low flow alert
		if ((lf_alert_state_flg==0) && (mains_status==1) && (low_flow_status==1) && low_flow_off_flg==1)
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			lf_alert_state_flg=1;
			low_flow_off_flg=0;
			
			lff_status=1;
			
			Run_State=0;
			stop_all_valve();
			HAL_Delay(500);

			pub_live_msg();
		}
		//=== low flow error correct alert during recharge state 1 and before recharge state 2 compliation
		else if ((lf_alert_state_flg==1) && (mains_status==1) && (low_flow_status==0))
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			lf_alert_state_flg=0;
			low_flow_off_flg=0;
		
			lff_status=0;
			
			HAL_Delay(500);
			pub_live_msg();
		}
		
		//=== val status
		if (mains_status==1)
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			Run_State=0;
			//======= loop try
			val_SS_M[0] = (val_M == 1) ? 'H' : 'L';

			if (val_M == 1) 
			{
				Run_State=1;
			} 
			
			// PB (vv_[2] to vv_[7]) ? val_SS_PB[0..5]
			for (int i = 0; i < 6; i++) 
			{
				val_SS_PB[i] = (vv_[i + 2] != 0) ? 'H' : 'L';
			
				if (Run_State==0 && vv_[i + 2] != 0) 
				{
						Run_State=1;
						//break;
				}
			}

			// PA (vv_[8] to vv_[15]) ? val_SS_PA[0..7]
			for (int i = 0; i < 8; i++) 
			{
				val_SS_PA[i] = (vv_[i + 8] != 0) ? 'H' : 'L';
					
				if (Run_State==0 && vv_[i + 8] != 0) 
				{
						Run_State=1;
						//break;
				}
			}
			
			// low flow reset --> for every startup till pump init time
			//if (Run_State==0)
			if (val_M==0)
			{
				pit_count=0;
				pit_count_up_flg=0;
				
				low_flow_off_flg=0;
				//lf_alert_state_flg=0; // try in 5 sec timer
			}

			memset(lcd_data, 0x00, sizeof(lcd_data));
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			sprintf(lcd_data, "%c1 %c2 %c3 %c4 %c5", val_SS_M[0], val_SS_PB[0], val_SS_PB[1], val_SS_PB[2], val_SS_PB[3]);
			LCD_Puts(lcd_data);
			LCD_Gotoxy(0, 1);
			sprintf(lcd_data, "%c6 %c7 %c8 %c9 %c10", val_SS_PB[4], val_SS_PB[5], val_SS_PA[0], val_SS_PA[1], val_SS_PA[2]);
			LCD_Puts(lcd_data);
			HAL_Delay(800);

			memset(lcd_data, 0x00, sizeof(lcd_data));
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			sprintf(lcd_data, "%c11 %c12 %c13 %c14", val_SS_PA[3],val_SS_PA[4],val_SS_PA[5],val_SS_PA[6]);
			LCD_Puts(lcd_data);
			LCD_Gotoxy(0, 1);
			sprintf(lcd_data, "%c15", val_SS_PA[7]);
			LCD_Puts(lcd_data);
			HAL_Delay(800);
			
			memset(lcd_data, 0x00, sizeof(lcd_data));
		}
		

			
		//=== after reconnect send live pkt in 10 sec
		if ((mains_status==1) && (live_pkt_reconnect_flag==1) && (duration_tick_flag_B==duration_tick))
		{
			//&&**live_values();
			pub_live_msg();
			duration_tick_flag_B=0;
			live_pkt_reconnect_flag=0;
			
			//stat_er_count++; // temp, comment it // only to test
		}
		
		//=== stat error chk during Run_state=1 -- (on running) -- after 20sec from (valve state chg)
		// to check mqtt conn --> value 3 --> if found stat error then set flg --> to excute reconn-fn =====
		// modem_reboot_count_on_mains!=2
		if ((mains_status==1) && (Sim_detected_flag==1) && (duration_tick_flag==duration_tick) && (modem_reboot_count_on_mains!=2))		
		{
			//memset(stat_err_buff, 0x00, sizeof(stat_err_buff));
			//HAL_Delay(100);	
			HAL_IWDG_Refresh(&hiwdg); // zz7
			memcpy(stat_err_buff,&Reply[0],150);
			HAL_Delay(100);
			//stat_error_chk_fn (stat_err_buff, "+QMTSTAT:");
			//stat_error_chk_fn (stat_err_buff, "ERROR    ERROR");
			stat_error_chk_fn ();
			
			if (stat_error_chk_flag==1 || stat_error_chk_flag==2)
			{
				//stat_er_count++; // temp, comment it // only to test
				
				stat_error_chk_flag=0;
				
				//memset(Reply,0,ECBuffer);	// will do by reconnect		
				reconnect_cn=0;
				minute_counter_mqtt_check_flag = 1;
			}
			duration_tick_flag=0; 
		}
		
		//=== live pkt --> after mains off alert ====
		if (mains_status==0 && modem_restart_flag==0)
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			HAL_Delay(100);
			Run_State=0;
			stop_all_valve();
			HAL_Delay(800);
			//&&**live_values();
			pub_live_msg();
			HAL_Delay(500);
			
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			LCD_Puts("MAINS OFF");
			HAL_Delay(1000);
			
			//=================================
			//if (val_M==0)
			//{
				pit_count=0;
				pit_count_up_flg=0;
				
				low_flow_off_flg=0;
			//}
			//=================================
			
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			stop_modem();
			HAL_Delay(1000);
			
			memset(Reply,0,1000);
			HAL_IWDG_Refresh(&hiwdg); // zz7
			modem_restart_flag=1;
			
			uart_reset_timer=0;
			no_sim_counter=0;
			
		}
		
		//======= error msg display==============
		// mains off msg
		if (mains_status==0)
		{
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			LCD_Puts("MAINS OFF");
			HAL_Delay(600);
		}
		
		if (mains_status==1 && rain_status==1)
		{
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			LCD_Puts("RAIN ERROR");
			HAL_Delay(600);
		}
		
		//if (mains_status==1 && low_flow_status==1)
		if (mains_status==1 && lff_status==1)
		{
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			LCD_Puts("LOW FLOW ERROR");
			HAL_Delay(600);
		}	
		//========error msg end==================
		
		//=== modem restart --> after main come back form mains_status=0
		if (mains_status==1 && modem_restart_flag==1)
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			HAL_Delay(1000);
			LCD_Init();
			lcd_on();
			reset_lcd_init_timer();		
			HAL_Delay(500);

			LCD_Clear();
			LCD_Gotoxy(0, 0);
			LCD_Puts("MAINS RETURN");
			LCD_Gotoxy(0, 1);
			LCD_Puts("MODEM RESTART");
			HAL_Delay(100);
			HAL_IWDG_Refresh(&hiwdg); // zz7
			start_modem();
			HAL_IWDG_Refresh(&hiwdg); // zz7
			HAL_Delay(2000);
						
			HAL_UART_DeInit(&huart2);
			HAL_UART_Init(&huart2);
			HAL_UART_Receive_DMA (&huart2, (uint8_t *)Reply,ECBuffer);
			
			HAL_IWDG_Refresh(&hiwdg); // zz7
			HAL_Delay(2000);
			start_modem_config(); // modem restart due to mains returns
			
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			if (lf_read_disable_flg!=1)
			{
				low_flow_status1 = HAL_GPIO_ReadPin(LOW_FLOW_GPIO_Port, LOW_FLOW_Pin); // main_status=1 but low flow status not get updated, so here reading it 
				low_flow_status = low_flow_status1;
				
				if(low_flow_status==0)
				{
					lff_status=0;
					//try
					lf_alert_state_flg=0;
					low_flow_off_flg=0;
				}
			}
			
			HAL_Delay(1000);
			pub_live_msg();

			HAL_IWDG_Refresh(&hiwdg); // zz7
			modem_restart_flag=0;

			// modem reboot due to mains off
			if ((nw_err_validity_flag==0) && (Sim_detected_flag==1))
			{
				// enable reconnecting --> due to mobile network found
				// to find cloud conn [3] --> till modem_reboot_count_on_mains=2
				// if sim detected but set ip is wrong and can't connet mqtt --> so, to prevent continuous modem on/off
				modem_reboot_count_on_mains=1; // for reconnecting , from 2 to 1 , --> modem reboot only one times
				reconnect_cn=0; // value append in 2min timer -- so after main_on --> assign 0
			}
			else if (nw_err_validity_flag==1)
			{
				// mobile network not found, so disable reconnecting
				modem_reboot_count_on_mains=2; // 
				reconnect_cn=6; // 
			}
	
		}
		

		if ((mains_status==1) && (HAL_GPIO_ReadPin(SW_2_GPIO_Port, SW_2_Pin) == 0) && (HAL_GPIO_ReadPin(SW_5_GPIO_Port, SW_5_Pin) == 0))
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			sw2=0;

			signal_strength_flag=1;
			

			if (Sim_detected_flag==0)
			{
				LCD_Clear();
				LCD_Gotoxy(0, 0);
				LCD_Puts("SIM NOT");
				LCD_Gotoxy(0, 1);
				LCD_Puts("DETECTED");
				signal_strength_flag=0;
				HAL_Delay(1000);
			}
			else if (nw_err_validity_boot_flag==1)
			{
				LCD_Clear();
				LCD_Gotoxy(0, 0);
				LCD_Puts("MOBILE NETWORK");
				LCD_Gotoxy(0, 1);
				LCD_Puts("NOT-GET AT BOOT");
				
				signal_strength_flag=0;
				HAL_Delay(1000);
			}
			else if (modem_reboot_count_on_mains==2 && nw_err_validity_boot_flag==0)
			{
				LCD_Clear();
				LCD_Gotoxy(0, 0);
				LCD_Puts("NO NETWORK");
				LCD_Gotoxy(0, 1);
				LCD_Puts("MODEM TRIED 2");
				
				signal_strength_flag=0;
				HAL_Delay(1000);
			}
			

			
			//&&**live_values();

		}

		
		if ((mains_status==1) && (HAL_GPIO_ReadPin(SW_3_GPIO_Port, SW_3_Pin) == 0) && (HAL_GPIO_ReadPin(SW_5_GPIO_Port, SW_5_Pin) == 0))
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
				sw1=0;
				sw2=0;
				sw3=0;
			
			if (modem_reboot_count_on_mains==2)
			{

				reconnect_cn=0;
				modem_reboot_count_on_mains=1;
				minute_counter_mqtt_check_flag = 1;
				Sim_detected_flag=1;
				
				LCD_Clear();
				LCD_Gotoxy(0, 0);
				LCD_Puts("RESUME NETWORK");
				LCD_Gotoxy(0, 1);
				LCD_Puts("RECONNECTING");
				HAL_Delay(2000);
			}
			else
			{
				pub_live_msg();
				HAL_Delay(1000);
			}
			
			//&&**live_values();
			
		}
		
		
		//=== mqtt conn check and retry on failure 
		if((mains_status==1) && (Sim_detected_flag==1) && (minute_counter_mqtt_check_flag==1))  // ori
		{ 
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			memset(buff_Check_conn, 0x00, sizeof(buff_Check_conn)); 
			
			Head_data = ECBuffer - hdma_usart2_rx.Instance->CNDTR;
			check_mqtt_status();
			Tail_data = ECBuffer - hdma_usart2_rx.Instance->CNDTR;
			
			HAL_Delay(200); // 25n
			//HAL_Delay(500);
		//	for(int i =Head_data ; i<=Tail_data;i++){
				memcpy(buff_Check_conn, &Reply[Head_data],(Tail_data-Head_data));
		//	}
				if(buff_Check_conn[14]=='3'){
					mqtt_connected_flag = 0;
					reconnect_cn=0;
					duration_tick_flag_B=(duration_tick+2); // ## after reconnect send live pkt in 10-sec 
					modem_reboot_count_on_mains=0; // modem reboot count clear --at 2 it stop modem_reboot on mains
					nw_err_validity_flag=0;
					nw_err_validity_boot_flag=0;
					no_cloud_val_off_flg=0;
					no_sim_counter=0;
	
				}
				else{
					mqtt_connected_flag = 1;
					reconnect_cn++;
					live_pkt_reconnect_flag=1; //## if conn fail then set flg, so that live pkt will be send after reconnect
						// atleast boot pkts send ok and time get ok-- but conn disconnect for 2 hours-- 38=76m -- this try reconnect again 
						//if ((reconnect_cn>=62) && (modem_restart_flag==0) && (retry_flg==0) && (time_ok==1))
						// in 16 min
						//if ((reconnect_cn>=8) && (modem_restart_flag==0) && (sim_problem==0))
						if ((reconnect_cn>=8) && (modem_restart_flag==0))
						{
								modem_reboot_count_on_mains=1; // for reconnecting , from 2 to 1 , --> modem reboot only one times
								reconnect_cn=0; // value append in 2min timer -- so after main_on --> assign 0
						}
				}
			minute_counter_mqtt_check_flag = 0;
			}
				
		//===============SIGNAL STRENGTH=======================
		if((mains_status==1) && (signal_strength_flag==1))
		{
			sig_strength();	
				
//			if (Run_State==1)
//			{
//				//&&**live_values(); // so that, in running state, lcd display text not stuck -- NETWORK STRENGTH
//			}
			
			signal_strength_flag = 0;
		}
			
				
		
		if((mains_status==1)&&(mqtt_connected_flag==1) && (reconnect_cn<=4) && (modem_reboot_count_on_mains!=2)) // mqtt reconnect
		{	
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			HAL_Delay(200);
			mqtt_reconnect();
			mqtt_connected_flag = 0;
			minute_counter_mqtt_check_flag = 1;
			
			LCD_Clear();
			LCD_Gotoxy(0, 1);
			LCD_Puts("reconnecting");
			HAL_Delay(800);
		}
		else if ((mains_status==1) && (reconnect_cn==5) && (modem_reboot_count_on_mains<=1)) // modem_reboot_count=1 --> trying 2nd time
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			reconnect_cn=0;
			mqtt_connected_flag = 0;
			minute_counter_mqtt_check_flag=0;
			
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			LCD_Puts("CONN");
			//HAL_Delay(100);
			LCD_Gotoxy(0, 1);
			LCD_Puts("RECONNECTING");
			HAL_Delay(1000);
			
			HAL_IWDG_Refresh(&hiwdg); // zz7
			stop_modem();
			HAL_IWDG_Refresh(&hiwdg); // zz7
			HAL_Delay(3000);
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			start_modem();
			HAL_IWDG_Refresh(&hiwdg); // zz7
			HAL_Delay(2000);
			
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			HAL_UART_DeInit(&huart2);
			HAL_UART_Init(&huart2);
			HAL_UART_Receive_DMA (&huart2, (uint8_t *)Reply,ECBuffer);
			
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			HAL_Delay(2000);
			start_modem_config(); // due to no cloud conn [3], after reconnect_cn=5 
			
			HAL_IWDG_Refresh(&hiwdg); // zz7
			

			
			HAL_Delay(500);
			//&&**live_values(); // to remove stuck char
			pub_live_msg();
			
			// 0-->1st try, 1-->2nd try --> stop modem reboot
			modem_reboot_count_on_mains++; // if sim=1 but set ip wrong and can't connet mqtt cloud --> so, to prevent continuous modem reboot
			
		}
		else if ((mains_status==1) && ((nw_err_validity_boot_flag==1) || (nw_err_validity_flag==1)) && (Sim_detected_flag==1))
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			LCD_Puts("MOBILE NETWORK");
			LCD_Gotoxy(0, 1);
			LCD_Puts("NOT FOUND");
			HAL_Delay(1000);
		}
		else if ((mains_status==1) && (modem_reboot_count_on_mains==2) && (Sim_detected_flag==1))
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			// if modem reboot 2 time but no cloud connection found --> for precaution stop valve 
			if (no_cloud_val_off_flg==0)
			{
				no_cloud_val_off_flg=1; // this get 0 again -- if sim detected and cloud connection made again
				HAL_Delay(100);
				stop_all_valve();
				HAL_Delay(800);
			}
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			LCD_Puts("CLOUD CONN");
			LCD_Gotoxy(0, 1);
			LCD_Puts("NOT FOUND");
			HAL_Delay(1000);
		}
		else if ((mains_status==1) && (Sim_detected_flag==0))
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			// if sim not get detected --> for precaution stop valve 
			if (no_cloud_val_off_flg==0)
			{
				no_cloud_val_off_flg=1; // this not get 0 again --> until sim detected in reboot/modem-reboot 
				HAL_Delay(100);
				stop_all_valve();
				HAL_Delay(800);
			}
			
			LCD_Clear();
			LCD_Gotoxy(0, 0);
			LCD_Puts("SIM NOT");
			LCD_Gotoxy(0, 1);
			LCD_Puts("DETECTED");
			signal_strength_flag=0;
			HAL_Delay(1000);
		}

		
//==============================sub data====
		CNDTR_Buff_check = ECBuffer - hdma_usart2_rx.Instance->CNDTR;    // check the buffer 
		//if(CNDTR_Buff_check >=100)                    //ori             // clear DMA buffer
		if(CNDTR_Buff_check >=73)
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			//memcpy(check_sub_data,&Reply[CNDTR_Buff_check_previous],40);
			if(sub_receive_flag == 0)
			{
				cntr_reset_flg=1; // new_mqtt
				
				//== ori
//				HAL_UART_DeInit(&huart2);
//				HAL_UART_Init(&huart2);
//			  HAL_UART_Receive_DMA (&huart2, (uint8_t *)Reply,ECBuffer);
				
				memset(check_sub_data, 0x00, sizeof(check_sub_data));
			}
			
			//HAL_Delay(500); // ori
			//HAL_Delay(200); // for bt
			memcpy(check_sub_data,&Reply[CNDTR_Buff_check_previous],150);
			check_sub_data_fn ();
			
			
		 if(sub_receive_flag == 1)
			{
				sub_receive_flag = 0;
				
				HAL_IWDG_Refresh(&hiwdg); // zz7
				
				memset(j_str, 0x00, sizeof(j_str)); // new_imp
				
				//memcpy(j_str,&Reply[CNDTR_Buff_check_previous+key_idx+40],700); //ori
				//memcpy(j_str,&Reply[CNDTR_Buff_check_previous+key_idx+40],41);
				memcpy(j_str,&Reply[CNDTR_Buff_check_previous+key_idx],700); // qos problem
						
				// to check the j_data valdity 
				//j_str_validity_fn (j_str, "\"wb\":");
				j_str_validity_fn ("\"v\":[");
				j_str_validity_fn ("]}");
				
				//if (j_str[0]=='{' && j_str_validity_flag==1)
				if (j_str_validity_flag==1 && j_str_validity_flag_2==1)
				{
					j_str_validity_flag=0;
					j_str_validity_flag_2=0;
					remote_pg_store=1; // flag-->to store the pg
				}
				else
				{
					memset(j_str, 0x00, sizeof(j_str));
					j_str_validity_flag=0;
					j_str_validity_flag_2=0;
					
					HAL_UART_DeInit(&huart2);
					HAL_UART_Init(&huart2);
					HAL_UART_Receive_DMA (&huart2, (uint8_t *)Reply,ECBuffer);
					sub_receive_flag = 0;
					HAL_Delay(500);
				}
				
				//memset(Reply,0,400); // new-imp
				//memset(check_sub_data, 0x00, sizeof(check_sub_data)); // new-imp
			}
			memset(Reply,0,700); // new-imp
			memset(check_sub_data, 0x00, sizeof(check_sub_data)); // new-imp
		}
		
		if (mains_status==1)
		{
			if (CNDTR_Buff_check==CNDTR_Buff_check_previous)
			{
				if (timer_uart_chk_flg==1)
				{
					uart_reset_timer++;
				}
				timer_uart_chk_flg=0;
				
				if (uart_reset_flg==1)
				{
					uart_reset_flg=0;
					
					HAL_IWDG_Refresh(&hiwdg); // zz7
					
					HAL_UART_DeInit(&huart2);
					HAL_UART_Init(&huart2);
					HAL_UART_Receive_DMA (&huart2, (uint8_t *)Reply,ECBuffer);
					CNDTR_Buff_check = ECBuffer - hdma_usart2_rx.Instance->CNDTR; 
					CNDTR_Buff_check_previous = CNDTR_Buff_check;
					
					uart_reset_timer=0;
				
					memset(Reply,0,1000);
				}
				
			}
			else if (CNDTR_Buff_check!=CNDTR_Buff_check_previous)
			{
				uart_reset_timer=0;
				uart_reset_flg=0;
			}
		}
		
		CNDTR_Buff_check_previous = CNDTR_Buff_check;
		
		if ((uart_reset_timer>=8) && (mains_status==1))
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			pub_live_msg();
			uart_reset_flg=1;
			uart_reset_timer=0;
		}
		
//===============remote config done=====
		if (remote_pg_store==1 && halt_alloperation==0)
		{
			remote_pg_store=0;
			
			HAL_IWDG_Refresh(&hiwdg); // zz7
			
			if (mains_status==1)
			{
				mqtt_val_parser ("\"v\":[");
				
				mqtt_valve_output();
			}
		}


		//pub_msg_sch(); // new position
		
		
		
		//if(CNDTR_Buff_check >=73)                                 // ori
		if((CNDTR_Buff_check >=73) || (cntr_reset_flg==1))                                 // clear DMA buffer
		{
				cntr_reset_flg=0;
			
				HAL_UART_DeInit(&huart2);
				HAL_UART_Init(&huart2);
			  HAL_UART_Receive_DMA (&huart2, (uint8_t *)Reply,ECBuffer);
				
				CNDTR_Buff_check = ECBuffer - hdma_usart2_rx.Instance->CNDTR; 
				CNDTR_Buff_check_previous = CNDTR_Buff_check;
		}
		
		

		pub_msg_sch(); // old position <-- best
		



		//HAL_Delay(100); // for bt
		if ((HAL_GPIO_ReadPin(SW_1_GPIO_Port, SW_1_Pin) == 0) && (HAL_GPIO_ReadPin(SW_2_GPIO_Port, SW_2_Pin) == 0))
		{
			HAL_IWDG_Refresh(&hiwdg); // zz7
			menu_flg=1;
			index1=1; // new_2_4
			sw1 = 0;
			sw2 = 0;
			run_mode = 0;
			mode_flag = 1;
			LCD_Clear();
			LCD_Puts("PLEASE WAIT..           ");
			HAL_Delay(1000);

			while ((mode_flag == 1) && (input_timeout < 100) && (index1 == 1))
			{
				HAL_IWDG_Refresh(&hiwdg); // zz7
				
				if ((sw1 == SW_START) && (input_timeout < SW_DELAY))
				{
					mode = pgm_mode[sw2];
					sw2_max = 8;
					LCD_Clear();
					LCD_Puts("SELECT MODE:      ");
					LCD_Gotoxy(0, 1);
					sprintf(msg, ": %s                 ", pgm_mode_disp[sw2]);
					LCD_Puts(msg);
					HAL_Delay(500);
				}
				else if ((sw1 == SW_START + 1) && (mode == 1))
				{

					sw1 = 0;
					sw2 = 0;

						HAL_IWDG_Refresh(&hiwdg); // zz7
						LCD_Clear();
						LCD_Gotoxy(0, 0);
						LCD_Puts("TEST MODE       ");
						LCD_Gotoxy(0, 1);
						LCD_Puts("                ");
						HAL_Delay(1000);
						test_mode();
					
				}
				else if ((sw1 == SW_START + 1) && (mode == 2))
				{
					//	sw1=0;
					index1++;
					sw2 = 0;
					LCD_Clear();
					set_date_day_time();
				}				
				else if ((sw1 == SW_START + 1) && (mode == 3))
				{
					sw2 = 0;
					cloudSetting();
					//&&**live_values();
					break;
				}
				else if ((sw1 == SW_START + 1) && (mode == 4))
				{
					sw2 = 0;
					PASS_CODE();
					if (sys_fn_flg==1)
					{
						cloud_set_menu();
						MenuDoneCnt=0; //extra care
						//&&**live_values();
						break;
					}
					//&&**live_values();
					break;
				}
					
				else if ((sw1 == SW_START + 1) && (mode == 5))
				{
					HAL_IWDG_Refresh(&hiwdg); // zz7
					
					if (mains_status==0)
					{
						LCD_Clear();
						LCD_Gotoxy(0, 0);
						LCD_Puts("ERROR");
						LCD_Gotoxy(0, 1);
						LCD_Puts("MAINS OFF");
						HAL_Delay(5000);
						HAL_IWDG_Refresh(&hiwdg); // zz7
						
						LCD_Clear();
						LCD_Gotoxy(0, 0);
						LCD_Puts("FIRST DO");
						LCD_Gotoxy(0, 1);
						LCD_Puts("MODEM RESTART");
						HAL_Delay(5000);
						HAL_IWDG_Refresh(&hiwdg); // zz7
					}
					
					////dig
					sw2 = 0;
					dig(); // for diagnostic
					//&&**live_values();
					break;
				}
				else if ((sw1 == SW_START + 1) && (mode == 6))
				{
					HAL_IWDG_Refresh(&hiwdg); // zz7
					// sensor config
					sw2 = 0;
					sensor_setting();
					//live_values();
					break;
				}
				else if ((sw1 == SW_START + 1) && (mode == 7))
				{
					// restart
					HAL_NVIC_SystemReset();
				}

				else if ((sw1 == SW_START + 1) && (mode == 8))
				{
					// exit
					break;
				}

				else
				{
					sw1 = SW_START;
				}
			} // menu while end
		} // menu if end


		if (menu_flg==1)
		{
			menu_flg=0;
			memset(Reply,0,1000);			
		}
	
		// when modem rebooting completed 
		if (mains_status==1 && modem_restart_flag==0)
		{
			uart_ERROR=HAL_UART_GetError(&huart2);         
			if(uart_ERROR!=0)
			{	
				data_error(&huart2);
				MX_USART2_UART_Init();	
			}
		}
		
	} // main while end 
	
  /* USER CODE END 3 */
}

// ori
//#if 0
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
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  //RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
//#endif

#if 0
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
//  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_HIGH);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
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
//  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}
#endif

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
	//get_hrs = *buff_time1_hh;
//	  sTime.Hours = *buff_time1_hh;
//  sTime.Minutes = *buff_time1_min;
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
//	  sDate.Month = *buff_date_yy;
//  sDate.Date = *buff_date_yy;
//  sDate.Year = *buff_date_yy;

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
  //htim6.Init.Prescaler = 32000-1; //ori
	htim6.Init.Prescaler = 16000-1;
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
  //htim14.Init.Prescaler = 32000-1; //ori
	htim14.Init.Prescaler = 16000-1;
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
 // HAL_GPIO_WritePin(LCD_LED_GPIO_Port, LCD_LED_Pin, GPIO_PIN_RESET);

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



	GPIO_InitStruct.Pin = MODEM_REG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	
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

//  /*Configure GPIO pin : LCD_LED_Pin */ ++$
  GPIO_InitStruct.Pin = LCD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_LED_GPIO_Port, &GPIO_InitStruct);

/*Configure GPIO pin : LCD_LED_Pin */
  GPIO_InitStruct.Pin = PWR_MODEM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWR_MODEM_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}


void mqtt_valve_output()
{
	//mcp_trigger_count++; // ^^test
	
	HAL_GPIO_WritePin(RELAY_PWR_GPIO_Port,RELAY_PWR_Pin,0);
	HAL_Delay(100);
	
	if (get_val_M==1) // vv_1
	{
		HAL_GPIO_WritePin(PUMP_R_GPIO_Port,PUMP_R_Pin,1);
		val_M=1;
	}
	else if (get_val_M==0)
	{
		HAL_GPIO_WritePin(PUMP_R_GPIO_Port,PUMP_R_Pin,0);
		val_M=0;
	}

	if (val_PB[0] == 1)      vv_[2] = 0x02;
	else if (val_PB[0] == 0) vv_[2] = 0x00;  
	// if val_PB[0] == 2 ? no change

	if (val_PB[1] == 1)      vv_[3] = 0x04;
	else if (val_PB[1] == 0) vv_[3] = 0x00;

	if (val_PB[2] == 1)      vv_[4] = 0x08;
	else if (val_PB[2] == 0) vv_[4] = 0x00;

	if (val_PB[3] == 1)      vv_[5] = 0x10;
	else if (val_PB[3] == 0) vv_[5] = 0x00;

	if (val_PB[4] == 1)      vv_[6] = 0x20;
	else if (val_PB[4] == 0) vv_[6] = 0x00;

	if (val_PB[5] == 1)      vv_[7] = 0x40;
	else if (val_PB[5] == 0) vv_[7] = 0x00;


	// Same for PA[]
	if (val_PA[0] == 1)      vv_[8]  = 0x20;
	else if (val_PA[0] == 0) vv_[8]  = 0x00;

	if (val_PA[1] == 1)      vv_[9]  = 0x40;
	else if (val_PA[1] == 0) vv_[9]  = 0x00;

	if (val_PA[2] == 1)      vv_[10] = 0x80;
	else if (val_PA[2] == 0) vv_[10] = 0x00;

	if (val_PA[3] == 1)      vv_[11] = 0x10;
	else if (val_PA[3] == 0) vv_[11] = 0x00;

	if (val_PA[4] == 1)      vv_[12] = 0x08;
	else if (val_PA[4] == 0) vv_[12] = 0x00;

	if (val_PA[5] == 1)      vv_[13] = 0x01;
	else if (val_PA[5] == 0) vv_[13] = 0x00;

	if (val_PA[6] == 1)      vv_[14] = 0x02;
	else if (val_PA[6] == 0) vv_[14] = 0x00;

	if (val_PA[7] == 1)      vv_[15] = 0x04;
	else if (val_PA[7] == 0) vv_[15] = 0x00;

	mcp_return=MCP_PB_WRITE(vv_[2]|vv_[3]|vv_[4]|vv_[5]|vv_[6]|vv_[7]);
	mcp_return=MCP_PA_WRITE(vv_[8]|vv_[9]|vv_[10]|vv_[11]|vv_[12]|vv_[13]|vv_[14]|vv_[15]);
	
	if (mcp_return==0)
	{
		pub_msg_confirmed();
		status_chg_flg=1;
	}
	
}

/* USER CODE BEGIN 4 */

void lcd_on()
{

	HAL_GPIO_WritePin(LCD_LED_GPIO_Port, LCD_LED_Pin, 1);

	lcd_on_flag = 1;
	lcd_timeout = 0;
}

void lcd_off()
{
	HAL_GPIO_WritePin(LCD_LED_GPIO_Port, LCD_LED_Pin, 0);

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
