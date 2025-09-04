#include "mcp23017.h"


extern I2C_HandleTypeDef hi2c2;
extern uint8_t mcp_return;
extern uint8_t mcp_mode_return;
extern uint8_t mcp_pre_val_A;
extern uint8_t mcp_pre_val_B;

uint8_t MCP_PA_WRITE(uint8_t value)
{
	
	mcp_return=HAL_I2C_Mem_Write(&hi2c2, MCP_ADDR, MCP_PORTA_REG, 1, &value, 1, 100);
	HAL_Delay(30);
	
	if (mcp_return!=0)
	{
		mcp_return=HAL_I2C_Mem_Write(&hi2c2, MCP_ADDR, MCP_PORTA_REG, 1, &value, 1, 100);
		HAL_Delay(30);
	}

	mcp_pre_val_A=value;
	return mcp_return;	
}



uint8_t MCP_PB_WRITE(uint8_t value)
{
	
	mcp_return=HAL_I2C_Mem_Write(&hi2c2, MCP_ADDR, MCP_PORTB_REG, 1, &value, 1, 100);
	HAL_Delay(30);
	
	if (mcp_return!=0)
	{
		mcp_return=HAL_I2C_Mem_Write(&hi2c2, MCP_ADDR, MCP_PORTB_REG, 1, &value, 1, 100);
		HAL_Delay(30);
	}

	mcp_pre_val_B=value;
	return mcp_return;	
}


uint8_t MCP_PA_OUT_MODE()
{
	uint8_t out_data = 0x00;
	
	
	mcp_mode_return=HAL_I2C_Mem_Write(&hi2c2, MCP_ADDR, MCP_PORTA, 1, &out_data, 1, 100);
	HAL_Delay(10);
	
	if (mcp_mode_return!=0)
	{
		mcp_mode_return=HAL_I2C_Mem_Write(&hi2c2, MCP_ADDR, MCP_PORTA, 1, &out_data, 1, 100);
		HAL_Delay(30);
	}

	return mcp_mode_return;
}

uint8_t MCP_PA_IN_MODE()
{
	uint8_t in_data = 0xFF;
	
	mcp_mode_return=HAL_I2C_Mem_Write(&hi2c2, MCP_ADDR, MCP_PORTA, 1, &in_data, 1, 100);
	HAL_Delay(10);
	
	if (mcp_mode_return!=0)
	{
		mcp_mode_return=HAL_I2C_Mem_Write(&hi2c2, MCP_ADDR, MCP_PORTA, 1, &in_data, 1, 100);
		HAL_Delay(30);
	}

	return mcp_mode_return;
}

uint8_t MCP_PB_OUT_MODE()
{
	uint8_t out_data = 0x00;
	
	mcp_mode_return=HAL_I2C_Mem_Write(&hi2c2, MCP_ADDR, MCP_PORTB, 1, &out_data, 1, 100);
	HAL_Delay(10);
	
	if (mcp_mode_return!=0)
	{
		mcp_mode_return=HAL_I2C_Mem_Write(&hi2c2, MCP_ADDR, MCP_PORTB, 1, &out_data, 1, 100);
		HAL_Delay(30);
	}

	return mcp_mode_return;
}


uint8_t MCP_PB_IN_MODE()
{
	uint8_t in_data = 0xFF;
	
	mcp_mode_return=HAL_I2C_Mem_Write(&hi2c2, MCP_ADDR, MCP_PORTB, 1, &in_data, 1, 100);
	HAL_Delay(10);
	
	if (mcp_mode_return!=0)
	{
		mcp_mode_return=HAL_I2C_Mem_Write(&hi2c2, MCP_ADDR, MCP_PORTB, 1, &in_data, 1, 100);
		HAL_Delay(30);
	}

	return mcp_mode_return;
}

uint8_t MCP_PA_READ()
{
	uint8_t buf[1] = {0};
	
	mcp_return=HAL_I2C_Mem_Read(&hi2c2, MCP_ADDR, MCP_PORTA_REG, 1, buf, 1, 100);
	HAL_Delay(30);
	
	if (mcp_return!=0)
	{
		mcp_return=HAL_I2C_Mem_Read(&hi2c2, MCP_ADDR, MCP_PORTA_REG, 1, buf, 1, 100);
		HAL_Delay(30);
	}

	return buf[0];	
}

uint8_t MCP_PB_READ()
{
	uint8_t buf[1] = {0};
	
	mcp_return=HAL_I2C_Mem_Read(&hi2c2, MCP_ADDR, MCP_PORTB_REG, 1, buf, 1, 100);
	HAL_Delay(30);
	
	if (mcp_return!=0)
	{
		mcp_return=HAL_I2C_Mem_Read(&hi2c2, MCP_ADDR, MCP_PORTB_REG, 1, buf, 1, 100);
		HAL_Delay(30);
	}

	return buf[0];	
}


void mcp_init()
{
	uint8_t off_data = 0x00;
	
	mcp_mode_return=MCP_PA_OUT_MODE();
	mcp_mode_return=MCP_PB_OUT_MODE();
	
	HAL_I2C_Mem_Write(&hi2c2, MCP_ADDR, MCP_PORTA_REG, 1, &off_data, 1, 100);
	HAL_Delay(10);
	HAL_I2C_Mem_Write(&hi2c2, MCP_ADDR, MCP_PORTB_REG, 1, &off_data, 1, 100);
	HAL_Delay(10);
}



