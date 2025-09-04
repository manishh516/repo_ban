#include "main.h"
#include <string.h>
#include <stdio.h> 
#include "ec_200_lib.h"
#include "lcd_16x2.h"

// extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

//char Test[4] = "AT\r\n";  
char Test[6] = "ATE0\r\n";                //Test the GSM module. Reply "OK" Running "ERROR" not working5  //FOR TESTING 
char Sim_identity[9] ="AT+CIMI\r\n";						//get International Mobile Subscriber Identity
char Stext[11] = "AT+CMGF=1\r\n";         //Set GSM in text mode
char get_network[10]="AT+COPS?\r\n";					// Query NETWORK 
//char get_network[14]="AT+COPS=0\r\n";
char Sstrength[8] = "AT+CSQ\r\n";        //signal strength
char check_Network[10] = "AT+CREG?\r\n";   //check network reg   
char get_sim_time[11]="AT+QLTS=2\r\n";				// QUERY CURRENT LOCAL TIME 

char mqtt_check[13] = "AT+QMTCONN?\r\n";

//Bluetooth 

//char BT_ON[13] = "AT+QBTPWR=4\r\n";
//char BT_OFF[13] = "AT+QBTPWR=0\r\n";
////char BT_[13] = "AT+QBTPWR=0\r\n";
////char BT_SET_NAME[15] = "AT+QBTNAME?\r\n";
//char BT_SET_NAME[24] = "AT+QBTNAME=0,\"LCA\"\r\n"; 
//char BT_CONNECTED[30] = "AT+QBTSEND=2,0,\"BTCONNECT\"\r\n";
//char BT_SCANMODE[20] = "AT+QBTSCANMODE=3\r\n";
//char TestFW[5] = "ATI\r\n";

//bluetooth add on by abhinav 2/6/25
char BT_OFF[13] = "AT+QBTPWR=0\r\n";
char BT_PWR[13] = "AT+QBTPWR=1\r\n";
char BT_ADV_PARA[35] = "AT+QBTGATADV=1,128,160,0,0,7,0\r\n";
//char BT_ADV_DATA[45] = "AT+QBTADVDATA=9,\"020106050938393130\"\r\n";	// ori
char BT_ADV_DATA[75]={0}; // ##
char BT_GATT_SRVC[25] = "AT+QBTGATSS=0,1,6159,1\r\n";
char BT_GATT_CHR[30] = "AT+QBTGATSC=0,0,58,1,10777\r\n";
char BT_CONF_CHR[40] = "AT+QBTGATSCV=0,0,3,1,10777,244,\"1234\"\r\n";
char BT_CHR_DESC[40] = "AT+QBTGATSCD=0,0,3,1,10498,2,\"0300\"\r\n";
char BT_CLR_SRVC[55] = "AT+QBTGATSS=1,0,\"2bbd25c43204425daebbbd401b2c58ef\",1\r\n";
//char BT_SEND_CMD_TO_PHONE[40] = "AT+QBTGATSNOD=0,0,18,4,\"07111111\"\r\n";
char BT_SEND_CMD_TO_PHONE[40] ={0};
char BT_pkt_to_phone[500]={0}; // bt_pkt_send ##


//repeat
//char BT_GATT_SRVC2[60] = "AT+QBTGATSC=1,0,58,0,\"f5899b5f8000008000100000FEFF1111\"";
//char BT_CONF_CHR2[70] = "AT+QBTGATSCV=1,1,3,0,\"f5899b5f8000008000100000FDFF1111\",244,\"1234\"";
//char BT_CHR_DESC2[40] = "AT+QBTGATSCD=1,1,3,1,10498,2,\"0300\"";
char BT_CLR_SRVC2[20] = "AT+QBTGATSSC=1,1\r\n";
char BT_CLR_ADV2[15] = "AT+QBTADV=1\r\n";

//char userpwd[] = "AT+QMTCONN=0,\"deployment-b0b6ae6b\",\"UCT\",\"uct@12345\"\r\n";  //Deployment name of emqx server, user ID and Password
// act--[37],, [40]
//char userpwd[37] = "AT+QMTCONN=0,\"lca20\",\r\n";  //Deployment name of emqx server, user ID and Password
//char userpwd[37]={0};
char userpwd[75]={0};

//char topic1[] = "AT+QMTSUB=0,1,\"lca2/sub/000000000000000\",1\r\n";        //subscribe to topic1
char topic1[45]={0};
	
//char pub[] = "AT+QMTPUBEX=0,1,1,0,\"topic1\",17\r\n";    //publish
char pub[70]={0};
//char result[70]={0}; // ## for topic -- pub // nnnmmm

char keepAlive[35]="AT+QMTCFG=\"keepalive\",0,90\r\n";

//char msgg[] = "message from EC20";                        //message // nnnmmm
//char end[] = "SUB";                                      //Ctrl-Z indicates end of SMS  // nnnmmm
//char close[] = "AT+QMTCLOSE=0\r\n";                      //to close ec20 module // nnnmmm

char stop_echo[] = "ATE0\r\n";                      //stop the echo
//char Configure_GNSS[] = "AT+QGPSCFG=?\r\n";         //Configure GNSS  // nnnmmm
//char Turn_on_GNSS[] = "ATE0\r\n";                   //Turn on GNSS
//char test_GPS_location[] = "AT+QGPSLOC=?\r\n";      //test GPS location
//char read_GPS_location[] = "AT+QGPSLOC=2\r\n";      //read the GPS location
//char Turn_OFF_GPS[] = "AT+QGPSEND\r\n";             //Turn OFF the GPS
char at_imei[8]="AT+GSN\r\n";
//char setapn[] = "AT+QICSGP=1,1,\"www\",\"\",\"\",1\r\n";                 //set apn
//char setapn[60] = "AT+QICSGP=1,1,\"airtelgprs.com\",\"\",\"\",1\r\n";                 //set apn
//char setapn[55] = {0}; // old
char setapn[105] = {0};
char emqx[60] = "AT+QMTOPEN=0,\"15.206.24.163\",1883\r\n";      //Connect Address of emqx server
//uint16_t len; // nnnmmm
char send_msg[220]={0};
//size_t msg_len=0;
uint16_t msg_len=0;


extern uint8_t Sim_detected_flag;

char Reply[ECBuffer];      //String to store reply by GSM module
char RxBuf[RxECBuffer];

uint8_t modem_state=0,cyclic_dma=0;
char imei_id[16];

HAL_StatusTypeDef state;



char topic_pub[25]="device/";      //lca2/up/IMEI
char topic_sub[25]="config/";

void setAPN(char *apnn) {

	sprintf(setapn, "AT+QICSGP=1,1,\"%s\",\"\",\"\",1\r\n", apnn);
	
//	memset(Reply,0,ECBuffer);		
	HAL_UART_Transmit_DMA (&huart2, (uint8_t *)setapn, strlen(setapn));	
//	HAL_UART_Receive_DMA (&huart2, (uint8_t *)Reply,ECBuffer); 	        //reply by GSM module
	//HAL_Delay(10); // old
	HAL_Delay(100);
	
}



void BT_OFF_FUN(void)	
{

	//HAL_UART_Transmit_DMA (&huart2, (uint8_t *)BT_OFF,strlen(BT_OFF));     //Send AT command  // change by prashant (10/06/25)

	HAL_UART_Transmit_DMA (&huart2, (uint8_t *)BT_OFF,sizeof(BT_OFF));  // 25n
	HAL_Delay(600);	
}

void BT_SEND_CMD_TO_PHONE_FUN(char *cmd_to_send){
	
	memset(BT_SEND_CMD_TO_PHONE, 0x00, sizeof(BT_SEND_CMD_TO_PHONE));
	
	sprintf(BT_SEND_CMD_TO_PHONE, "AT+QBTGATSNOD=0,0,18,4,\"%s\"\r\n", cmd_to_send);
	
	HAL_UART_Transmit_DMA (&huart2, (uint8_t *)BT_SEND_CMD_TO_PHONE, strlen(BT_SEND_CMD_TO_PHONE)); 
	
	HAL_Delay(300);	
}


// ori
#if 0
void BT_START_FUN(void)
{

		HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_PWR, sizeof(BT_PWR));
    HAL_Delay(1000);

    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_ADV_PARA, sizeof(BT_ADV_PARA));
    HAL_Delay(1000);

    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_ADV_DATA, sizeof(BT_ADV_DATA));    
    HAL_Delay(1000);

    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_GATT_SRVC, sizeof(BT_GATT_SRVC));    
    HAL_Delay(1000);

    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_GATT_CHR, sizeof(BT_GATT_CHR));	
    HAL_Delay(1000);

    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_CONF_CHR, sizeof(BT_CONF_CHR));   
    HAL_Delay(1000);

    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_CHR_DESC, sizeof(BT_CHR_DESC));   
    HAL_Delay(1000);

    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_CLR_SRVC, sizeof(BT_CLR_SRVC));   
    HAL_Delay(1000); 
		
		HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_CLR_SRVC2, sizeof(BT_CLR_SRVC2));   
    HAL_Delay(1000); 
		
		HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_CLR_ADV2, sizeof(BT_CLR_ADV2));   
    HAL_Delay(1000); 
	
}
#endif


void BT_START_FUN(void)
{

		HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_PWR, sizeof(BT_PWR));
    HAL_Delay(600);

    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_ADV_PARA, sizeof(BT_ADV_PARA));
    HAL_Delay(600);

		//sprintf(BT_ADV_DATA, "AT+QBTADVDATA=9,\"0201060509%02X%02X%02X%02X\"\r\n", (unsigned char)imei_id[11],(unsigned char)imei_id[12],(unsigned char)imei_id[13],(unsigned char)imei_id[14]);
		sprintf(BT_ADV_DATA, "AT+QBTADVDATA=17,\"0201060D094155544F4D41545F%02X%02X%02X%02X\"\r\n", (unsigned char)imei_id[11],(unsigned char)imei_id[12],(unsigned char)imei_id[13],(unsigned char)imei_id[14]);	
		HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_ADV_DATA, strlen(BT_ADV_DATA));    
    HAL_Delay(600);

    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_GATT_SRVC, sizeof(BT_GATT_SRVC));    
    HAL_Delay(600);

    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_GATT_CHR, sizeof(BT_GATT_CHR));	
    HAL_Delay(600);

    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_CONF_CHR, sizeof(BT_CONF_CHR));   
    HAL_Delay(600);

    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_CHR_DESC, sizeof(BT_CHR_DESC));   
    HAL_Delay(600);

    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_CLR_SRVC, sizeof(BT_CLR_SRVC));   
    HAL_Delay(600); 
		
		HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_CLR_SRVC2, sizeof(BT_CLR_SRVC2));   
    HAL_Delay(600); 
		
		HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_CLR_ADV2, sizeof(BT_CLR_ADV2));   
    HAL_Delay(600); 
	
}

// kkk
void BT_BOOT_START_FUN(void)
{

		HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_PWR, sizeof(BT_PWR));
    HAL_Delay(400);

    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_ADV_PARA, sizeof(BT_ADV_PARA));
    HAL_Delay(400);

		//sprintf(BT_ADV_DATA, "AT+QBTADVDATA=9,\"0201060509%02X%02X%02X%02X\"\r\n", (unsigned char)imei_id[11],(unsigned char)imei_id[12],(unsigned char)imei_id[13],(unsigned char)imei_id[14]);
    sprintf(BT_ADV_DATA, "AT+QBTADVDATA=17,\"0201060D094155544F4D41545F%02X%02X%02X%02X\"\r\n", (unsigned char)imei_id[11],(unsigned char)imei_id[12],(unsigned char)imei_id[13],(unsigned char)imei_id[14]);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_ADV_DATA, strlen(BT_ADV_DATA));    
    HAL_Delay(400);

    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_GATT_SRVC, sizeof(BT_GATT_SRVC));    
    HAL_Delay(400);

    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_GATT_CHR, sizeof(BT_GATT_CHR));	
    HAL_Delay(400);

    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_CONF_CHR, sizeof(BT_CONF_CHR));   
    HAL_Delay(400);

    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_CHR_DESC, sizeof(BT_CHR_DESC));   
    HAL_Delay(400);

    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_CLR_SRVC, sizeof(BT_CLR_SRVC));   
    HAL_Delay(400);
		
		HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_CLR_SRVC2, sizeof(BT_CLR_SRVC2));   
    HAL_Delay(400); 
		
		HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BT_CLR_ADV2, sizeof(BT_CLR_ADV2));   
    HAL_Delay(400); 
	
}

void set_userpwd(char *pwd) 
{
	//sprintf(userpwd, "AT+QMTCONN=0,\"lca_%s\"\r\n", pwd); // ori
	sprintf(userpwd, "AT+QMTCONN=0,\"lca_%s\",\"lca2\",\"smartIRRI#24\"\r\n", pwd);

  //memset(Reply,0,ECBuffer); //ori
	HAL_UART_Transmit_DMA (&huart2, (uint8_t *)userpwd, strlen(userpwd));
//	HAL_UART_Receive_DMA (&huart2, (uint8_t *)Reply,ECBuffer); 	        //reply by GSM module
  HAL_Delay(100);
	
}


void serverIP(char *ip, char *port) {

	memset(emqx, 0x00, sizeof(emqx));
	
	sprintf(emqx, "AT+QMTOPEN=0,\"%s\",%s\r\n", ip,port);
	
	//memset(Reply,0,ECBuffer);	//ori
	
	HAL_UART_Transmit_DMA (&huart2, (uint8_t *)emqx, strlen(emqx)); 
//	HAL_UART_Receive_DMA (&huart2, (uint8_t *)Reply,ECBuffer); 	        //reply by GSM module
	HAL_Delay(100);
	
}


#if 0
void pub_msg()
{

	// for msg_len
	msg_len = strlen(send_msg);
	
	sprintf(pub, "AT+QMTPUBEX=0,1,1,0," "\"%s\"," "%d\r\n", topic_pub,msg_len);
	
	//memset(Reply,0,ECBuffer);
	HAL_UART_Transmit_DMA (&huart2, (uint8_t *)pub, strlen(pub)); 
	HAL_Delay(200); //ori
	//HAL_Delay(500);
	
//	memset(Reply,0,ECBuffer);
	//HAL_UART_Transmit_DMA (&huart2, (uint8_t *)send_msg, strlen(send_msg)); // added the enter in the end
	HAL_UART_Transmit_DMA (&huart2, (uint8_t *)send_msg, msg_len);
	HAL_Delay(200);  //ori //25n
	
	

	//HAL_UART_Transmit_DMA (&huart2, (uint8_t *)enter, strlen(enter)); //ori
//  HAL_Delay(10);

}
#endif

// bt_pkt ##
void BT_pkt_send(char *bt_send_pkt)
{
	
	uint16_t bt_data_len=0; // 
	
	// to pkt length
	bt_data_len = strlen(bt_send_pkt)/2;
	
	memset(BT_pkt_to_phone, 0x00, sizeof(BT_pkt_to_phone));
	
	sprintf(BT_pkt_to_phone, "AT+QBTGATSNOD=0,0,18,%d,\"%s\"\r\n", bt_data_len, bt_send_pkt);
	
	HAL_UART_Transmit_DMA (&huart2, (uint8_t *)BT_pkt_to_phone, strlen(BT_pkt_to_phone)); 
	
	HAL_Delay(400);	
}


void pub_msg()
{

	// for msg_len
	msg_len = strlen(send_msg);
	
	//if (Sim_detected_flag==1)
	//{
		sprintf(pub, "AT+QMTPUBEX=0,1,1,0," "\"%s\"," "%d\r\n", topic_pub,msg_len);
		
		HAL_UART_Transmit_DMA (&huart2, (uint8_t *)pub, strlen(pub)); 
		HAL_Delay(200); //ori
		
		
		HAL_UART_Transmit_DMA (&huart2, (uint8_t *)send_msg, msg_len);
		HAL_Delay(200);  //ori //25n
	
	//}

}

void sub_topic(char *b) {

	sprintf(topic1, "AT+QMTSUB=0,1,\"%s\",1\r\n", b);
	
  //memset(Reply,0,ECBuffer); //ori		
	HAL_UART_Transmit_DMA (&huart2, (uint8_t *)topic1, strlen(topic1)); 	
//	HAL_UART_Receive_DMA (&huart2, (uint8_t *)Reply,ECBuffer); 	        //reply by GSM module
	HAL_Delay(100);
	
}


void testgsm(void)	
{
//	memset(Reply,0,ECBuffer);
	HAL_UART_Transmit_DMA (&huart2, (uint8_t *)Test,6);     //Send AT command  // change by prashant (10/06/25)
//	if(cyclic_dma==1)
	state=HAL_UART_Receive_DMA (&huart2, (uint8_t *)Reply,ECBuffer);
	
//	state=HAL_UARTEx_ReceiveToIdle_DMA  (&huart2, (uint8_t *)Reply,ECBuffer); 
	HAL_Delay(100);	
}


void get_imei(void)
{

//	memset(Reply,0,ECBuffer);
	// 	manipulation of dma index
//	hdma_usart2_rx.Instance->CNDTR=499U;
	
	HAL_UART_Transmit_DMA (&huart2, (uint8_t *)at_imei,7);     //Send Sim test command
//	HAL_UART_Receive_DMA (&huart2, (uint8_t *)rx5,100); 	   //reply by GSM module
//	HAL_UARTEx_ReceiveToIdle(&huart2, (uint8_t *)Reply,ECBuffer,&len,1000); 	   //reply by GSM module
//		HAL_UART_Receive(&huart2, (uint8_t *)Reply,ECBuffer,1000);
	if(cyclic_dma==1)
	{
		HAL_UART_Receive_DMA (&huart2, (uint8_t *)Reply,ECBuffer);
	}
	HAL_Delay(10);
		
}

void echo_off(void)
{
	int size=0;
	size=sizeof(at_imei);
	
		HAL_UART_Transmit_DMA (&huart2,(uint8_t *)stop_echo,size); 
	if(cyclic_dma==1)
	{
		state=HAL_UARTEx_ReceiveToIdle_DMA  (&huart2, (uint8_t *)Reply,ECBuffer); 
	}
		HAL_Delay(100);
}

void detect_sim(void)
{

//	memset(Reply,0,ECBuffer);
	HAL_UART_Transmit_DMA (&huart2, (uint8_t *)Sim_identity,9);     //Send Sim test command
	if(cyclic_dma==1)
	{
		state=HAL_UARTEx_ReceiveToIdle_DMA  (&huart2, (uint8_t *)Reply,ECBuffer); 
	}
//	state=HAL_UARTEx_ReceiveToIdle_DMA  (&huart2, (uint8_t *)Reply,ECBuffer); 
	HAL_Delay(200);
		
}

void check_network(void)
{

//	memset(Reply,0,ECBuffer);
	HAL_UART_Transmit_DMA (&huart2, (uint8_t *)check_Network,10);     //Send Sim test command
	if(cyclic_dma==1)
	{
		state=HAL_UARTEx_ReceiveToIdle_DMA  (&huart2, (uint8_t *)Reply,ECBuffer); 
	}
//	state=HAL_UARTEx_ReceiveToIdle_DMA  (&huart2, (uint8_t *)Reply,ECBuffer); 
	HAL_Delay(100);
	
}


void set_nw_auto(void)
{

//	memset(Reply,0,ECBuffer);
	HAL_UART_Transmit_DMA (&huart2, (uint8_t *)get_network,10);     //Send Sim test command
	if(cyclic_dma==1)
	{
		state=HAL_UARTEx_ReceiveToIdle_DMA  (&huart2, (uint8_t *)Reply,ECBuffer); 
	}
//	state=HAL_UARTEx_ReceiveToIdle_DMA  (&huart2, (uint8_t *)Reply,ECBuffer); 
	//HAL_Delay(10); //ori
	HAL_Delay(100);
}




void signal_strengt(void)
{

//	memset(Reply,0,ECBuffer);
	HAL_UART_Transmit_DMA (&huart2, (uint8_t *)Sstrength,8);     //Send Sim test command
	if(cyclic_dma==1)
	{
		HAL_UART_Receive_DMA (&huart2, (uint8_t *)Reply,ECBuffer);
	}
//	state=HAL_UARTEx_ReceiveToIdle_DMA  (&huart2, (uint8_t *)Reply,ECBuffer); 
	HAL_Delay(100);
	
}



void get_time_sim(void)
{

	//memset(Reply,0,ECBuffer);
	HAL_UART_Transmit_DMA (&huart2, (uint8_t *)get_sim_time,11);     //Send Sim test command
//	HAL_UART_Receive_DMA (&huart2, (uint8_t *)Reply,ECBuffer);	   //reply by GSM module
	if(cyclic_dma==1)
	{
//	HAL_UART_Receive_DMA (&huart2, (uint8_t *)Reply,ECBuffer);
	}

	HAL_Delay(100);
	
}


#if 0
void sub_mqtt_application(void)
{	
  memset(Reply,0,ECBuffer); 	
	HAL_UART_Transmit_DMA (&huart2, (uint8_t *)topic1, strlen(topic1)); 	
	if(cyclic_dma==1)
	{
		HAL_UART_Receive_DMA (&huart2, (uint8_t *)Reply,ECBuffer);
	}
	HAL_Delay(10);
}

void pub_mqtt_application(void)
{
	memset(Reply,0,ECBuffer);
	HAL_UART_Transmit_DMA (&huart2, (uint8_t *)pub, strlen(pub)); 
	HAL_Delay(10);
  
	memset(Reply,0,ECBuffer);
	HAL_UART_Transmit_DMA (&huart2, (uint8_t *)msgg, strlen(msgg));
	HAL_Delay(10);
	
	HAL_UART_Transmit_DMA (&huart2, (uint8_t *)end, strlen(end)); 
  HAL_Delay(10);
}

void turn_off_mqtt_application(void)
{
	memset(Reply,0,ECBuffer);
	HAL_UART_Transmit_DMA (&huart2, (uint8_t *)close, strlen(close));
  HAL_Delay(10);
}
#endif



void start_modem(void)
{
	if(modem_state==0)
	{
		HAL_GPIO_WritePin(MODEM_REG_GPIO_Port,MODEM_REG_Pin,0);
		HAL_Delay(1400);
		HAL_GPIO_WritePin(MODEM_REG_GPIO_Port,MODEM_REG_Pin,1);
		HAL_Delay(1400);
			
		HAL_GPIO_WritePin(PWR_MODEM_GPIO_Port,PWR_MODEM_Pin,0);
		HAL_Delay(2400);
		HAL_GPIO_WritePin(PWR_MODEM_GPIO_Port,PWR_MODEM_Pin,1);
		modem_state=1;
	}
	
}

void check_mqtt_status(){

	
  //memset(Reply,0,ECBuffer);
	//HAL_UART_Transmit_DMA (&huart2, (uint8_t *)mqtt_check, strlen(mqtt_check)); // ori -- bt problem with strlen
	HAL_UART_Transmit_DMA (&huart2, (uint8_t *)mqtt_check, sizeof(mqtt_check)); // 25n
//	HAL_UART_Receive_DMA (&huart2, (uint8_t *)Reply,ECBuffer); 	        //reply by GSM module
  HAL_Delay(200);
	
	
}



void stop_modem(void)
{
	if(modem_state==1)
	{
		HAL_GPIO_WritePin(MODEM_REG_GPIO_Port,MODEM_REG_Pin,0);
		HAL_Delay(1400);
		HAL_GPIO_WritePin(MODEM_REG_GPIO_Port,MODEM_REG_Pin,1);
		HAL_Delay(1400);
		
		HAL_GPIO_WritePin(PWR_MODEM_GPIO_Port,PWR_MODEM_Pin,0);
		HAL_Delay(2400);
		//HAL_GPIO_WritePin(PWR_MODEM_GPIO_Port,PWR_MODEM_Pin,1);
		modem_state=0;
		HAL_GPIO_WritePin(MODEM_REG_GPIO_Port,MODEM_REG_Pin,0);
	}
}


void keep_alive() {
	
 // memset(Reply,0,ECBuffer);
	//HAL_UART_Transmit_DMA (&huart2, (uint8_t *)keepAlive, strlen(keepAlive)); // ori -bt chg 
	HAL_UART_Transmit_DMA (&huart2, (uint8_t *)keepAlive, sizeof(keepAlive)); // 25n
  HAL_Delay(100);
}


#if 0
void ping_packet(char *ping_interval) {
	char a[] = "AT+QMTCFG=\"\"qmtping\",""0\",";
	char c[] = "\",\r\n";

	//size_t length = strlen(a) + strlen(ping_interval) + strlen(c); 

	//char result[length];

	strcpy(result, a);
	strcat(result, ping_interval);
	strcat(result, c);
	
	strcpy(userpwd, result);
	
  memset(Reply,0,ECBuffer);
	HAL_UART_Transmit_DMA (&huart2, (uint8_t *)userpwd, strlen(userpwd));
  HAL_Delay(10);
	
}



void to_check_SIM_status(void)
	{
		char to_check_ready[]="READY";
		char rx3[100]={0};
		char recieve_ready[10];
		HAL_Delay(5);
		char tx3[]={"AT+CPIN?\r\n"};
		rx3[0]='\0';
		uint8_t tx_len3=strlen(tx3); 
		HAL_UART_Transmit(&huart2,(uint8_t*)tx3,tx_len3,100);
	  HAL_UART_Receive(&huart2,(uint8_t*)rx3,1000, 100);  // Receiving data via usart
		if(rx3[0]!='\0')                               // checking if valid data exists or not
		  {
			    int len=strlen(rx3);                     
       
		    HAL_Delay(2);  
				rx3[0]='\0';
		  }
		recieve_ready[0]=rx3[18];
		recieve_ready[1]=rx3[19];
		recieve_ready[2]=rx3[20];
		recieve_ready[3]=rx3[21];
		recieve_ready[4]=rx3[22];
		    if(strcmp(recieve_ready,to_check_ready)==0)
		       {
			       
		           HAL_Delay(10);
					 }
		    else
		       {
			        
						   HAL_Delay(10);
		       }
				HAL_Delay(10);
		}
#endif
