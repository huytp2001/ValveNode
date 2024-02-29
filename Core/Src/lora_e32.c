/**
  ******************************************************************************
  * @file           : lora_e32.c
  * @brief          : 
  ******************************************************************************
  */
#include "main.h"
#include "lora_e32.h"

static bool b_lora_complete(uint16_t u16_timeout);

// Private function ---------------------------

e_lora_mode_t e_lora_set_mode(e_lora_mode_t e_mode)
{
	switch(e_mode)
	{
		case MODE_NORMAL:
		{
			HAL_GPIO_WritePin(RF_M0_GPIO_Port, RF_M0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RF_M1_GPIO_Port, RF_M1_Pin, GPIO_PIN_RESET);
		}
		break;
	  case MODE_WAKEUP:
		{
			HAL_GPIO_WritePin(RF_M0_GPIO_Port, RF_M0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RF_M1_GPIO_Port, RF_M1_Pin, GPIO_PIN_RESET);
		}
		break;
		case MODE_POWERDOWN:
		{
			HAL_GPIO_WritePin(RF_M0_GPIO_Port, RF_M0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RF_M1_GPIO_Port, RF_M1_Pin, GPIO_PIN_SET);
		}
		break;
		case MODE_PROGRAM:
		{
			HAL_GPIO_WritePin(RF_M0_GPIO_Port, RF_M0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RF_M1_GPIO_Port, RF_M1_Pin, GPIO_PIN_SET);
		}
		break;
		default: 
			break;
	}
	HAL_Delay(PIN_RECOVER);
	return e_mode;
}

uint8_t u8_lora_set_sped(uint8_t u8_parity_2bit, uint8_t u8_uart_bps_3bit, uint8_t u8_air_bps_3bit)
{
	uint8_t u8_sped = 0;
	u8_sped = ((u8_parity_2bit & 0x03) << 6) | ((u8_uart_bps_3bit & 0x07) << 3) | (u8_air_bps_3bit & 0x07);
	return u8_sped;
}

uint8_t u8_lora_set_option(bool b_fm_bit, bool b_io_driver_bit, uint8_t u8_wakeup_t_3bit, bool b_fec_bit, uint8_t u8_opt_2bit)
{
	uint8_t u8_option = 0;
	u8_option = ((b_fm_bit & 0x01) << 7) | ((b_io_driver_bit & 0x01) << 6) | ((u8_wakeup_t_3bit & 0x07) << 3) | ((b_fec_bit & 0x01) << 2) | (u8_opt_2bit & 0x03);
	return u8_option;
}

bool b_lora_save_parameter(uint8_t u8_save_mode, uint8_t u8_high_addr, uint8_t u8_low_addr, uint8_t u8_sped, uint8_t u8_channel, uint8_t u8_options)
{
	bool b_success = false;
	e_lora_set_mode(MODE_PROGRAM);
	uint8_t au8_save_buf[6] = {u8_save_mode, u8_high_addr, u8_low_addr, u8_sped, u8_channel, u8_options};
	uart2_transmit(au8_save_buf, 6);
	HAL_Delay(PIN_RECOVER);
	b_success = b_lora_complete(5000);
	e_lora_set_mode(MODE_NORMAL);
	return b_success;
}

bool b_lora_send_data(uint8_t u8_high_addr, uint8_t u8_low_addr, uint8_t u8_channel, uint8_t* pu8_buffer, uint8_t u8_len_buf)
{
	HAL_GPIO_WritePin(STT_GPIO_Port, STT_Pin, GPIO_PIN_SET);
	bool b_success = false;
	uart2_transmit(&u8_high_addr, 1);
	uart2_transmit(&u8_low_addr, 1);
	uart2_transmit(&u8_channel, 1);
	uart2_transmit(pu8_buffer, u8_len_buf);
	HAL_Delay(PIN_RECOVER);
	b_success = b_lora_complete(1000);
	HAL_GPIO_WritePin(STT_GPIO_Port, STT_Pin, GPIO_PIN_RESET);
	return b_success;
}

uint8_t u8_lora_received_data(uint8_t* pu8_buffer)
{
	uint8_t u8_len = 0;
	if(uart2_available() == true)
	{
		u8_len = uart2_get_data(pu8_buffer);
		// check lenght
		if(u8_len != pu8_buffer[0])
		{
			u8_len = 0;
		}
	}
	return u8_len;
}

// Static function ---------------------------

static bool b_lora_complete(uint16_t u16_timeout)
{
	uint16_t u16_counter = 0;
	bool b_complete = true;
	while (HAL_GPIO_ReadPin(RF_AUX_GPIO_Port, RF_AUX_Pin) == GPIO_PIN_RESET)
	{
		HAL_Delay(1);
		u16_counter++;
		if (u16_counter >= u16_timeout)
		{
			b_complete = false;
			break;
		}
	}
	return b_complete;
}
