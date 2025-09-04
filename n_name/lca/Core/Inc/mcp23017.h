#ifndef __MCP23017_H
#define __MCP23017_H

#include "main.h" 


#define MCP_ADDR 0x40
#define MCP_PORTA 0x00
#define MCP_PORTB 0x01


#define MCP_PORTA_REG 0x12
#define MCP_PORTB_REG 0x13

#define MCP_5 0x01
#define MCP_6 0x02
#define MCP_7 0x04
#define MCP_8 0x08
#define MCP_9 0x10
#define MCP_10 0x20
#define MCP_11 0x40
#define MCP_12 0x80

#define MCP_ALL_ON 0xFF
#define MCP_ALL_OFF 0x00


uint8_t MCP_PA_WRITE(uint8_t value);
uint8_t MCP_PB_WRITE(uint8_t value);

uint8_t MCP_PA_OUT_MODE();
uint8_t MCP_PA_IN_MODE();
uint8_t MCP_PB_OUT_MODE();
uint8_t MCP_PB_IN_MODE();


uint8_t MCP_PA_READ();
uint8_t MCP_PB_READ();

void mcp_init();


#endif
