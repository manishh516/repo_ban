#ifndef __PKT_PUB_H
#define __PKT_PUB_H
#include "main.h"


//mqtt_msg_with_topics
void pub_boot_msg();
void pub_live_msg();

void pub_msg_confirmed();

void apn_msg_confirmed();

void apn_pkt();

void pkt_t_msg_confirmed();

void rst_msg_confirmed();

void rns_msg_confirmed();
void lfs_msg_confirmed();
void cwm_msg_confirmed();
void pit_msg_confirmed();

#endif
