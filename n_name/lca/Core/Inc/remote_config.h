#ifndef __REMOTE_CONFIG_H
#define __REMOTE_CONFIG_H

#include "main.h"
#include <string.h>
#include <stdio.h> 
#include "ec_200_lib.h"

uint16_t one_digit_parser_2nd_config (char *key_str);

void bt_apn_parser (char *key_str); // iot apn

void mqtt_val_parser (char *key_str);

#endif

