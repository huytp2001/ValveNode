/**
  ******************************************************************************
  * @file           : setup_address.c
  * @brief          : 
  ******************************************************************************
  */
#include "main.h"
#include "setup_address.h"

static uint8_t u8_h_address = 0;
static uint8_t u8_l_address = 0;
static uint8_t u8_channel = 0;

void u8_address_setup(void)
{
	// Enable SW
	HAL_GPIO_WritePin(EN_SEN_GPIO_Port, EN_SEN_Pin, GPIO_PIN_SET);
	// Read SW
	HAL_Delay(50);
	u8_h_address = 1
							 + HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) * 1
							 + HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) * 2 
							 + HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) * 4; // 1~8
							 
	u8_l_address = 1
							 + HAL_GPIO_ReadPin(SW4_GPIO_Port, SW4_Pin) * 1
							 + HAL_GPIO_ReadPin(SW5_GPIO_Port, SW5_Pin) * 2
							 + HAL_GPIO_ReadPin(SW6_GPIO_Port, SW6_Pin) * 4; // 1~8
	
	if (HAL_GPIO_ReadPin(SW7_GPIO_Port, SW7_Pin) == GPIO_PIN_RESET)
	{
		u8_channel 	 = ADDR_CHAN_868MHZ;
	}
	else
	{
		u8_channel 	 = ADDR_CHAN_915MHZ;
	}
	// Disable SW
	HAL_GPIO_WritePin(EN_SEN_GPIO_Port, EN_SEN_Pin, GPIO_PIN_RESET);
}

uint8_t u8_address_get_high(void)
{
	return u8_h_address;
}

uint8_t u8_address_get_low(void)
{
	return u8_l_address;
}

uint8_t u8_address_get_chan(void)
{
	return u8_channel;
}
