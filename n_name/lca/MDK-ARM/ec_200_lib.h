#ifndef __EC_200_LIB_H
#define __EC_200_LIB_H
#include "main.h"


#define ECBuffer 1000
#define RxECBuffer 14

void check_mqtt_status();
//void mqtt_reconnect();
void start_modem(void);
void stop_modem(void);
void testgsm(void);	
//void start_mqtt_application(void); // nnnmmm
//void pub_mqtt_application(void); // nnnmmm
//void sub_mqtt_application(void); // nnnmmm
//void turn_off_mqtt_application(void); // nnnmmm
void detect_sim(void);
void check_network(void);
void set_nw_auto(void);
void signal_strengt(void);
void get_time_sim(void);
void get_imei(void);
void echo_off(void);
void sub_topic(char *b);
void keep_alive();

//Bluetooth code
//void BT_ON_FUN(void)	;
//void BT_OFF_FUN(void)	;
//void BT_SET_NAME_FUN(void)	;
//void BT_CONNECTED_FUN(void)	;
//void BT_SCANMODE_FUN(void)	;

void BT_OFF_FUN(void)	;
//void TestFW_FUN(void)	;
void BT_START_FUN(void);
void BT_SEND_CMD_TO_PHONE_FUN(char *cmd_to_send);

void BT_BOOT_START_FUN(void); // kkk


extern char Reply[ECBuffer];      //String to store reply by GSM module
extern char RxBuf[RxECBuffer];
//extern char imei_id[15]="12345678912345";  //imei id
extern char imei_id[16];

extern char topic_pub[25];      //lca2/up/IMEI
extern char topic_sub[25];

void setAPN(char *apn);



//void pub_msg(char *topic, char *msg); //ori
void pub_msg();

void serverIP(char *ip, char *port);
void set_userpwd(char *pwd);


//void start_gps_application(void); // nnnmmm
//void test_gps_application(void); // nnnmmm
//void turn_off_gps_application(void); // nnnmmm
//void to_check_SIM_status(void); // nnnmmm

#endif
