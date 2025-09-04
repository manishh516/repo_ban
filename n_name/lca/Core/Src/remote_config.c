#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "ec_200_lib.h"

extern uint8_t get_val_M;
extern uint8_t val_PB[6];
extern uint8_t val_PA[8];

extern uint8_t effectiveFromToday;
extern uint8_t get_exp;
extern uint8_t get_total_v;
extern uint8_t irrigationMode[2];
extern uint16_t masterSchdValveDurationMins[48];
extern uint16_t masterSchdPreWetDurMins[48];
extern uint16_t masterSchdFertgDurMins[48];
extern uint16_t masterSchdFlushingDurMins[48];
extern uint16_t masterSchdPumpSttimeMins[4];
extern uint8_t weekdaysPgmA;
extern uint8_t weekdaysPgmB;
extern uint8_t pumpInitiationTimeSecs;
extern uint16_t pumpRechargingTimeAbsMins;

extern char check_sub_data[150];
extern char extracted_buffer5[500]; // for bt em pkt
extern char apn[65];

extern int get_date, get_month, get_year, get_hrs, get_mins, get_day; // // bt_time

uint16_t ex_var[24]={0};
char extract_str[130] = {0};
uint8_t idxx = 0;
uint8_t op_br_idx=0;


uint8_t z=0; //for loop

// for mode_parser
uint16_t ex_vari[4]={0};

// for bt_time_parser
uint16_t bt_time_val[6]={0}; // bt_time

// for st_parser
uint16_t ex_vars[4]={0};

// for one_digit_parser
uint16_t ex_varr=0; // due to pump recharge time 
uint8_t colon_idx=0;


// pgA irrg
//char j_str[350] = "{\"ut\": 1,\"ee\": 1,\"vc\": 5,\"m\": [1,0],\"vd\": [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],\"pwd\": [1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4],\"frtd\": [1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4],\"fd\": [1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4],\"st\": [1,0,0,0],\"wa\": 1,\"wb\": 0}";

//pgB irrg
//char j_str[350] = "{\"ut\": 1,\"ee\": 1,\"vc\": 5,\"m\": [0,1],\"vd\": [1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],\"pwd\": [1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4],\"frtd\": [1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4],\"fd\": [1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4],\"st\": [0,0,375,380],\"wa\": 0,\"wb\": 1,\"it\": 5,\"rt\": 1}";

//double irrg
//char j_str[350] = "{\"ut\": 1,\"ee\": 1,\"vc\": 5,\"m\": [1,1],\"vd\": [1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],\"pwd\": [1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4],\"frtd\": [1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4],\"fd\": [1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4],\"st\": [300,305,310,315],\"wa\": 1,\"wb\": 1}";

//char j_str[350]="{\"ut\":1,\"ee\":1,\"vc\":12,\"m\":[0,2],\"vd\":[3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3],\"pwd\":[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],\"frtd\":[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],\"fd\":[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],\"st\":[0,0,1200,1300],\"wa\":0,\"wb\":1}";

//double fert
char j_str[700]={0};


uint16_t one_digit_parser_2nd_config (char *key_str)
{
//    char extract_str[30] = {0};
//    uint8_t idxx = 0;
//    uint16_t ex_varr=0; // due to pump recharge time 
//    uint8_t colon_idx=0;
	
		memset(extract_str, 0x00, sizeof(extract_str));
		idxx = 0;
    ex_varr=0; // due to pump recharge time 
    colon_idx=0;

		
    for(z=0; z<10; z++)
    {
        if (key_str[z] == ':') 
        {
					colon_idx=z+1; //exact : index -- \"ut\":
					break;
        }
    }
    
    
    // locate the start of the "vd" char
    char *start = strstr(check_sub_data, key_str);
    if (start) 
		{
			start += colon_idx; // move past the :
  
    
        // copy characters until the comma ,
        while (*start != ',' && *start != '\0') 
				{
            extract_str[idxx++] = *start++; // idx 1-53
            //printf("nn- %d\n", idx); 
        }
        extract_str[idxx] = '\0'; // idx=53 --null terminator
        
        //printf("b_string - %s\n", b);
    
        // convert string-num to integer
        char *token = strtok(extract_str, ",}"); // 3\06\09\012,3,6,9,12,3,6,9,12,3,6,9,12,3,6,9,12,3,6,9,12
				
				ex_varr=atoi(token);

				//return ex_var;
    }
		return ex_varr;
    
}

void bt_apn_parser (char *key_str)
{
 
		memset(extract_str, 0x00, sizeof(extract_str));
		idxx = 0;
		op_br_idx=0; // to find the open bracket index
		
		memset(apn, 0x00, sizeof(apn));
	

    for(z=0; z<10; z++)
    {
        if (key_str[z] == '[')
        {
            op_br_idx=z+1;  //exact open bracket index
						break;
        }
    }
    
    // locate the key_str in j_str
    char *start = strstr(extracted_buffer5, key_str);
    if (start) 
		{
        start += op_br_idx; // move past the open bracket part --- "\"vd\": [" 
  
        
        // copy characters until the closing bracket ]
        while (*start != ']' && *start != '\0') {
            extract_str[idxx++] = *start++; // idx 1-53 
        }
        extract_str[idxx] = '\0'; // idx=53 --null terminator
        
				
        
        // Remove surrounding double quotes if present
				uint8_t ch_len = strlen(extract_str);
				if (extract_str[0] == '"' && extract_str[ch_len - 1] == '"') {
						memmove(extract_str, extract_str + 1, ch_len - 2);
						extract_str[ch_len - 2] = '\0'; // trim the ending quote
				}
				
				sprintf(apn, "%s", extract_str);
 
    } 
    
}


void mqtt_val_parser (char *key_str)
{
    //char extract_str[130] = {0};
	  //uint8_t idxx = 0;
		//uint8_t op_br_idx=0; // to find the open bracket index
	
		memset(extract_str, 0x00, sizeof(extract_str));
    idxx = 0;
    op_br_idx=0; // to find the open bracket index
    
		for(z=0; z<10; z++)
    {
        if (key_str[z] == '[')
        {
            op_br_idx=z+1;  //exact open bracket index
						break;
        }
    }
    
    // locate the key_str in j_str
    char *start = strstr(j_str, key_str);
    if (start) 
		{
        start += op_br_idx; // move past the open bracket part --- "\"vd\": [" 
  
        
        // copy characters until the closing bracket ]
        while (*start != ']' && *start != '\0') {
            extract_str[idxx++] = *start++; // idx 1-53 
        }
        extract_str[idxx] = '\0'; // idx=53 --null terminator
        
				
        
        // convert string-num to integer
        char *token = strtok(extract_str, ","); // 3\06\09\012,3,6,9,12,3,6,9,12,3,6,9,12,3,6,9,12,3,6,9,12
        idxx=0; // re-assign index to 0
        while (token != NULL) 
				{
					ex_var[idxx]=atoi(token);
					token = strtok(NULL, ",");
					idxx++;
        }
    } 
    
			get_val_M=ex_var[0];
		
			for(z=1; z<=6; z++)
			{
				val_PB[z-1]=ex_var[z];
			}
			
			for(z=0; z<=7; z++)
			{
				val_PA[z]=ex_var[z+7];
			}
				
}


