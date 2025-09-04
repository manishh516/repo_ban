#include "main.h"
#include <string.h>
#include <stdio.h> 
#include "ec_200_lib.h"

extern IWDG_HandleTypeDef hiwdg;

extern uint8_t pumpInitiationTimeSecs;
extern uint8_t lff_status;

extern char apn[65];
extern char send_msg[220];
extern uint8_t mains_status;
extern uint8_t Sim_detected_flag;
extern uint16_t custom_pkt_time;
extern int duration_tick_flag;
extern int duration_tick;

extern uint32_t Total_Flow1_count;
extern uint8_t TotalFlow1Counter;
extern uint8_t sensor_status;
extern uint8_t rain_status;
extern uint8_t low_flow_status;

extern uint8_t vv_[16];

extern uint8_t val_M;
extern uint8_t vv_2,vv_3,vv_4,vv_5,vv_6,vv_7,vv_8,vv_9,vv_10,vv_11,vv_12,vv_13,vv_14,vv_15;
extern uint8_t val_PB[6];
extern uint8_t val_PA[8];
extern uint8_t halt_alloperation;

uint8_t cntr_reset_flg=0;
uint32_t pkt_id=0;

void pub_live_msg()
{
	HAL_IWDG_Refresh(&hiwdg); // zz7
	
	pkt_id++;
	
	uint8_t type=1;	

	uint8_t ms=mains_status; //mains status
	
	uint8_t val_[16]= {0};  

	// Assign with loop
	for (uint8_t i = 2; i <= 15; i++) 
	{
		HAL_IWDG_Refresh(&hiwdg); // zz7
		val_[i] = (vv_[i] != 0) ? 1 : 0;
	}
	
//	if (ms==0)
//	{
//		
//	}
	
	memset(send_msg, 0x00, sizeof(send_msg));
	
	////===live_msg sprint
  sprintf(send_msg, "{\"id\": \"%s\"," 
	"\"d\": [%u,"
	"%u,"
	"%u,"
	"%u,"
	"%u,"
	"%u,"
	"%u,"
	"%u,"
	"%u,"
	"%u,"
	"%u,"
	"%u,"
	"%u,"
	"%u,"
	"%u,"
	"%u,"
	"%u,"
	"%u,"
	"%u,"
	"%u,"
	"%u,"
	"%u,"
	"%u,"
	"%u,"
	"%u,"
	"%u]}"
	"\r\n",
	imei_id,
	type,
	ms,
	custom_pkt_time,
	halt_alloperation,
	Total_Flow1_count,
	TotalFlow1Counter,
	rain_status,
	lff_status,
	sensor_status,
	pumpInitiationTimeSecs,
	pkt_id,
	val_M,
	val_[2],
	val_[3],
	val_[4],
	val_[5],
	val_[6],
	val_[7],
	val_[8],
	val_[9],
	val_[10],
	val_[11],
	val_[12],
	val_[13],
	val_[14],
	val_[15]);
	
	HAL_IWDG_Refresh(&hiwdg); // zz7
	if ((mains_status==1) && (Sim_detected_flag==1))
	{
		cntr_reset_flg=1;
	}
	
	HAL_IWDG_Refresh(&hiwdg); // zz7
	duration_tick_flag=(duration_tick+4);
	pub_msg();
}


// confirm msg
void pub_msg_confirmed(){
	
	memset(send_msg, 0x00, sizeof(send_msg));
	
	uint8_t type=2;
			sprintf(send_msg, "{\"id\": \"%s\"," 
		"\"d\": [%u," //type
		"1]}""\r\n",      //confirm 
		imei_id, 
		type
	);
	   
	pub_msg();
}


// apn confirm msg
void apn_msg_confirmed(){
	
	memset(send_msg, 0x00, sizeof(send_msg));
	
	uint8_t type=3;
			sprintf(send_msg, "{\"id\": \"%s\"," 
		"\"d\": [%u," //type
		"1]}""\r\n",      //confirm 
		imei_id, 
		type
	);
	   
	pub_msg();
}

// restart confirm msg
void rst_msg_confirmed(){
	
	memset(send_msg, 0x00, sizeof(send_msg));
	
	uint8_t type=5;
			sprintf(send_msg, "{\"id\": \"%s\"," 
		"\"d\": [%u," //type
		"1]}""\r\n",      //confirm 
		imei_id, 
		type
	);
	   
	pub_msg();
}

// apn confirm msg
void pub_boot_msg(){
	
	memset(send_msg, 0x00, sizeof(send_msg));
	
	uint8_t type=9;
			sprintf(send_msg, "{\"id\": \"%s\"," 
		"\"d\": [%u," //type
		"1]}""\r\n",      //confirm 
		imei_id, 
		type
	);
	   
	pub_msg();
}

// custom pkt time set confirm msg
void pkt_t_msg_confirmed(){
	
	memset(send_msg, 0x00, sizeof(send_msg));
	
	uint8_t type=4;
			sprintf(send_msg, "{\"id\": \"%s\"," 
		"\"d\": [%u," //type
		"1]}""\r\n",      //confirm 
		imei_id, 
		type
	);
	   
	pub_msg();
}


// rain sensor config confirm msg
void rns_msg_confirmed(){
	
	memset(send_msg, 0x00, sizeof(send_msg));
	
	uint8_t type=6;
			sprintf(send_msg, "{\"id\": \"%s\"," 
		"\"d\": [%u," //type
		"1]}""\r\n",      //confirm 
		imei_id, 
		type
	);
	   
	pub_msg();
}

// low flow sensor config confirm msg
void lfs_msg_confirmed(){
	
	memset(send_msg, 0x00, sizeof(send_msg));
	
	uint8_t type=7;
			sprintf(send_msg, "{\"id\": \"%s\"," 
		"\"d\": [%u," //type
		"1]}""\r\n",      //confirm 
		imei_id, 
		type
	);
	   
	pub_msg();
}

// clear water meter sensor config confirm msg
void cwm_msg_confirmed(){
	
	memset(send_msg, 0x00, sizeof(send_msg));
	
	uint8_t type=8;
			sprintf(send_msg, "{\"id\": \"%s\"," 
		"\"d\": [%u," //type
		"1]}""\r\n",      //confirm 
		imei_id, 
		type
	);
	   
	pub_msg();
}

// pit confirm msg
void pit_msg_confirmed(){
	
	memset(send_msg, 0x00, sizeof(send_msg));
	
	uint8_t type=10;
			sprintf(send_msg, "{\"id\": \"%s\"," 
		"\"d\": [%u," //type
		"1]}""\r\n",      //confirm 
		imei_id, 
		type
	);
	   
	pub_msg();
}


void apn_pkt()
{
	
	memset(send_msg, 0x00, sizeof(send_msg));
	
	uint8_t type=31;
	sprintf(send_msg, "{\"id\": \"%s\",""\"apn\": [\"%s\"]}""\r\n",       
	imei_id, 
	apn);
	   
	pub_msg();
}



