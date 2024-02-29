/**
  ******************************************************************************
  * @file           : control_valve.c
  * @brief          : 
  ******************************************************************************
  */
#include "main.h"
#include "control_valve.h"
#include "flash_mem.h"

static uint8_t u8_coil_state = 0x00;
static void v_ctrl_on(uint8_t port_on);
static void v_ctrl_off(uint8_t port_off);

void v_ctrl_coil_action(void)
{
	u8_coil_state = u8_flash_read(0) & 0x0F;
	// coding...4 coil in 1 valve station
	for(uint8_t i = 0; i < 4; i++)
	{
		if((u8_coil_state >> i) & 0x01)
		{
			v_ctrl_on(i+1);
		}
		else
		{
			v_ctrl_off(i+1);
		}
	}
}

void v_ctrl_set(uint8_t u8_state)
{
	u8_coil_state = u8_state;
	if (u8_coil_state != (u8_flash_read(0) & 0x0F))
	{
		u32_flash_write((uint8_t*) &u8_coil_state, 1);
	}
}

static void v_ctrl_on(uint8_t port_on)
{
	// ON  = cap discharge -> waiting 50ms -> charge    -> waiting 50ms
	switch (port_on)
	{
		case 1:
		{
			HAL_GPIO_WritePin(DIS_1_GPIO_Port, DIS_1_Pin, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(DIS_1_GPIO_Port, DIS_1_Pin, GPIO_PIN_RESET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(EN_1_GPIO_Port, EN_1_Pin, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(EN_1_GPIO_Port, EN_1_Pin, GPIO_PIN_RESET);
			HAL_Delay(100);
			// LED1 ON
			//HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		}
		break;
		case 2:
		{
			HAL_GPIO_WritePin(DIS_2_GPIO_Port, DIS_2_Pin, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(DIS_2_GPIO_Port, DIS_2_Pin, GPIO_PIN_RESET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(EN_2_GPIO_Port, EN_2_Pin, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(EN_2_GPIO_Port, EN_2_Pin, GPIO_PIN_RESET);
			HAL_Delay(100);
			// LED2 ON
			//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		}
		break;
		case 3:
		{
			HAL_GPIO_WritePin(DIS_3_GPIO_Port, DIS_3_Pin, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(DIS_3_GPIO_Port, DIS_3_Pin, GPIO_PIN_RESET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(EN_3_GPIO_Port, EN_3_Pin, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(EN_3_GPIO_Port, EN_3_Pin, GPIO_PIN_RESET);
			HAL_Delay(100);
			// LED3 ON
			//HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
		}
		break;
		case 4:
		{
			HAL_GPIO_WritePin(DIS_4_GPIO_Port, DIS_4_Pin, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(DIS_4_GPIO_Port, DIS_4_Pin, GPIO_PIN_RESET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(EN_4_GPIO_Port, EN_4_Pin, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(EN_4_GPIO_Port, EN_4_Pin, GPIO_PIN_RESET);
			HAL_Delay(100);
			// LED4 ON
			//HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
		}
		break;
		default:
		break;
	}
}

static void v_ctrl_off(uint8_t port_off)
{
	// OFF = cap charge    -> waiting 50ms -> discharge -> waiting 50ms
	switch (port_off)
	{
		case 1:
		{
			HAL_GPIO_WritePin(EN_1_GPIO_Port, EN_1_Pin, GPIO_PIN_SET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(EN_1_GPIO_Port, EN_1_Pin, GPIO_PIN_RESET);
			HAL_Delay(5);
			HAL_GPIO_WritePin(DIS_1_GPIO_Port, DIS_1_Pin, GPIO_PIN_SET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(DIS_1_GPIO_Port, DIS_1_Pin, GPIO_PIN_RESET);
			HAL_Delay(5);
			// LED1 OFF
			//HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		}
		break;
		case 2:
		{
			HAL_GPIO_WritePin(EN_2_GPIO_Port, EN_2_Pin, GPIO_PIN_SET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(EN_2_GPIO_Port, EN_2_Pin, GPIO_PIN_RESET);
			HAL_Delay(5);
			HAL_GPIO_WritePin(DIS_2_GPIO_Port, DIS_2_Pin, GPIO_PIN_SET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(DIS_2_GPIO_Port, DIS_2_Pin, GPIO_PIN_RESET);
			HAL_Delay(5);
			// LED2 OFF
			//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		}
		break;
		case 3:
		{
			HAL_GPIO_WritePin(EN_3_GPIO_Port, EN_3_Pin, GPIO_PIN_SET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(EN_3_GPIO_Port, EN_3_Pin, GPIO_PIN_RESET);
			HAL_Delay(5);
			HAL_GPIO_WritePin(DIS_3_GPIO_Port, DIS_3_Pin, GPIO_PIN_SET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(DIS_3_GPIO_Port, DIS_3_Pin, GPIO_PIN_RESET);
			HAL_Delay(5);
			// LED3 OFF
			//HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
		}
		break;
		case 4:
		{
			HAL_GPIO_WritePin(EN_4_GPIO_Port, EN_4_Pin, GPIO_PIN_SET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(EN_4_GPIO_Port, EN_4_Pin, GPIO_PIN_RESET);
			HAL_Delay(5);
			HAL_GPIO_WritePin(DIS_4_GPIO_Port, DIS_4_Pin, GPIO_PIN_SET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(DIS_4_GPIO_Port, DIS_4_Pin, GPIO_PIN_RESET);
			HAL_Delay(5);
			// LED4 OFF
			//HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
		}
		break;
		default:
		break;
	}
}
