/**
  ******************************************************************************
  * @file           : process_val.c
  * @brief          : 
  ******************************************************************************
  */
	
	
#include "main.h"
#include "process_valve.h"
#include "lora_e32.h"
#include "build_frame.h"
#include "setup_address.h"
#include "battery.h"
#include "control_valve.h"

#define INCREASE_DIRECTION 	true
#define DECREASE_DIRECTION 	false

static uint8_t au8_data_received[32];
static uint8_t au8_data_ctr_saved[32];
static uint8_t au8_data_rep_saved[32];

static uint8_t u8_valve_state = 0;
static uint8_t u8_try_send = 0;

static e_valve_waiting_t e_waiting_state = WAITING_CONTROL;
static stru_valve_addr_t stru_address;

static stru_frame_lora_t str_data_ctr;
static stru_frame_lora_t str_data_rep;

// Static function -----------------------------
static void v_process_valve_my_addr(void);
static uint8_t u8_process_count_next_station(uint8_t u8_routing_data);
static uint8_t u8_process_continuous_addr(uint8_t u8_routing_data, bool select_direction);
static void v_process_ack(uint8_t u8_h_addr_direct, uint8_t u8_l_addr_direct, bool select_direction);
static void v_process_repeat(void);
static void v_process_report(void);

void v_process_loop(void)
{
	memset(au8_data_received, 0x00, sizeof(au8_data_received));
	// Check uart
	uint8_t u8_uart_data_len = u8_lora_received_data(au8_data_received);
	if (u8_uart_data_len >= FRAME_LEN_ACK && u8_uart_data_len <= FRAME_LEN_INFO + 1)
	{
		u8_uart_data_len = 0;
		stru_frame_lora_t str_data;
		uint8_t u8_check_data = u8_frame_read(au8_data_received, &str_data);
		v_process_valve_my_addr();
		switch(u8_check_data)
		{
			case FRAME_TYPE_ID_CTRL:
			{
				memcpy(&str_data_ctr, &str_data, sizeof(str_data));
				stru_address.u8_prev_addr = u8_process_continuous_addr(str_data_ctr.u8_routing_val, DECREASE_DIRECTION);
				stru_address.u8_next_addr = u8_process_continuous_addr(str_data_ctr.u8_routing_val, INCREASE_DIRECTION);
				// Send frame alive to prev addr
				if ( stru_address.u8_prev_addr < stru_address.u8_low_addr 
					&& stru_address.u8_prev_addr != 0)
				{
					v_process_ack(stru_address.u8_high_addr, stru_address.u8_prev_addr, DECREASE_DIRECTION);
				}
				else 
				{
					v_process_ack(stru_address.u8_high_addr, 0x00, DECREASE_DIRECTION);
				}
				// Send frame control to next addr
				if (0 != strncmp((char*) au8_data_received, (char*) au8_data_ctr_saved, au8_data_received[0]))
				{
					memcpy(au8_data_ctr_saved, au8_data_received, sizeof(au8_data_received));
					if ((str_data_ctr.u8_en_state_val >> (stru_address.u8_low_addr - 1)) & 1)
					{
						u8_valve_state = (str_data_ctr.u32_state_val >> 4*(stru_address.u8_low_addr - 1)) & 0x0F;
					}
					v_ctrl_set(u8_valve_state);
					v_process_repeat();
				}
			}
			break;
			case FRAME_TYPE_ID_REPS:
			{
				memcpy(&str_data_rep, &str_data, sizeof(str_data));														
				// Valve is NOT End Node
				stru_address.u8_next_addr = u8_process_continuous_addr(str_data_rep.u8_routing_val, INCREASE_DIRECTION);
				stru_address.u8_prev_addr = u8_process_continuous_addr(str_data_rep.u8_routing_val, DECREASE_DIRECTION);
				// Send frame alive to next addr
				if ( stru_address.u8_next_addr > stru_address.u8_low_addr 
					&& stru_address.u8_next_addr != 0)
				{
					v_process_ack(stru_address.u8_high_addr, stru_address.u8_next_addr, INCREASE_DIRECTION);
				}
				// Send frame rep to prev addr
				if (e_waiting_state == WAITING_REPLY)
				{
					if (0 != strncmp((char*) au8_data_received, (char*) au8_data_rep_saved, au8_data_received[0]))
					{
						memcpy(au8_data_rep_saved, au8_data_received, sizeof(au8_data_received));
						v_process_report();
					}
				}
			}
			break;
			case FRAME_TYPE_NEXT_ACK:
			{
				if(e_waiting_state == WAITING_NEXT_ACK)
				{
					// Wait rep frame
					v_timer_reset_couter();
					u8_try_send = 0;
					e_waiting_state = WAITING_REPLY;
				}
			}
			break;
			case FRAME_TYPE_PREV_ACK:
			{
				if(e_waiting_state == WAITING_PREV_ACK)
				{
					// Finish
					v_timer_reset_couter();
					u8_try_send = 0;
					e_waiting_state = WAITING_CONTROL;
				}
			}
			break;
			case FRAME_ERR_LEN:
			{
				// Check lengh false
			}
			break;
			case FRAME_ERR_SUM:
			{
				// Check sum false
			}
			break;
			case FRAME_ERR_ADDR:
			{
				// Check address false
			}
			break;
			default:
			break;
		}
	}
	// Check counter time
	switch(e_waiting_state)
	{
		case WAITING_CONTROL:
		{
			if (u16_timer_get_couter() > 3 * VALVE_STEP_IN_SEC)
			{
				v_timer_reset_couter();
				memset(au8_data_ctr_saved, 0x00, sizeof(au8_data_ctr_saved));
				memset(au8_data_rep_saved, 0x00, sizeof(au8_data_rep_saved));
				memset(&str_data_ctr, 0x00, sizeof(str_data_ctr));
				memset(&str_data_rep, 0x00, sizeof(str_data_ctr));
				v_ctrl_coil_action();
				HAL_GPIO_WritePin(EN_SEN_GPIO_Port, EN_SEN_Pin, GPIO_PIN_RESET);
				// Go to power save mode
				//e_lora_set_mode(MODE_POWERDOWN);
				//mcu_sleep();
				// Wakeup....
				//e_lora_set_mode(MODE_WAKEUP);
				v_timer_reset_couter();
				HAL_GPIO_WritePin(EN_SEN_GPIO_Port, EN_SEN_Pin, GPIO_PIN_SET);
				u8_try_send = 0;
			}
		}
		break;
		case WAITING_NEXT_ACK:
		{
			if (u16_timer_get_couter() > 3 * VALVE_STEP_IN_SEC)
			{
				v_timer_reset_couter();
				if (u8_try_send < TRY_SEND_MAX_NUM)
				{
					u8_try_send++;
					v_process_repeat();
				}
				else
				{
					u8_try_send = 0;
					// Change 'str_data_ctr.u32_routing_val'
					str_data_ctr.u8_routing_val = str_data_ctr.u8_routing_val & ~((uint8_t) 1 << (stru_address.u8_next_addr - 1));
					// Set new u8_next_addr
					stru_address.u8_next_addr = u8_process_continuous_addr(str_data_ctr.u8_routing_val, INCREASE_DIRECTION);
					v_process_repeat();
				}
			}
		}
		break;
		case WAITING_REPLY:
		{
			if (u16_timer_get_couter() > u8_process_count_next_station(str_data_ctr.u8_routing_val) * TRY_SEND_MAX_NUM * 4 * VALVE_STEP_IN_SEC)
			{
				v_timer_reset_couter();
				u8_try_send = 0;
				// Change 'str_data_ctr.u8_routing_val'
				str_data_ctr.u8_routing_val = str_data_ctr.u8_routing_val & ~((uint8_t) 1 << (stru_address.u8_next_addr - 1));
				// Set new u8_next_addr
				stru_address.u8_next_addr = u8_process_continuous_addr(str_data_ctr.u8_routing_val, INCREASE_DIRECTION);
				v_process_repeat();
			}
		}
		break;
		case WAITING_PREV_ACK:
		{
			if (u16_timer_get_couter() > 3 * VALVE_STEP_IN_SEC)
			{
				v_timer_reset_couter();
				if (u8_try_send < TRY_SEND_MAX_NUM)
				{
					u8_try_send++;
					v_process_report();
				}
				else
				{
					u8_try_send = 0;
					if ( stru_address.u8_prev_addr < stru_address.u8_low_addr 
						&& stru_address.u8_prev_addr != 0)
					{
						// Change 'str_data_rep.u8_routing_val'
						str_data_rep.u8_routing_val = str_data_rep.u8_routing_val & ~((uint8_t) 1 << (stru_address.u8_prev_addr - 1));
						// Set new u8_prev_addr
						stru_address.u8_prev_addr = u8_process_continuous_addr(str_data_rep.u8_routing_val, DECREASE_DIRECTION);
						v_process_report();
					}
					else
					{
						// Valve is Start Node - Finish
						v_timer_reset_couter();
						u8_try_send = 0;
						e_waiting_state = WAITING_CONTROL;
					}
				}
			}
		}
		break;
		default:
		{
			//mcu_sleep();
		}
		break;
	}
}

// -------------------------------------------------

static void v_process_valve_my_addr(void)
{
	stru_address.u8_high_addr = u8_address_get_high();
	stru_address.u8_low_addr = u8_address_get_low();
	stru_address.u8_channel = u8_address_get_chan();
}

static uint8_t u8_process_count_next_station(uint8_t u8_routing_data)
{
	uint8_t u8_couter = 1;
	uint8_t u8_addr = stru_address.u8_low_addr;
	while (((u8_routing_data >> (u8_addr + (u8_couter - 1))) & 1) == false)
	{
		u8_couter++;
		if (u8_couter + u8_addr >= 8)
		{
			u8_couter = 0;
			break;
		}
	}
	return u8_couter + 1;
}

static uint8_t u8_process_continuous_addr(uint8_t u8_routing_data, bool select_direction)
{
	uint8_t u8_couter = 1;
	uint8_t u8_addr = stru_address.u8_low_addr;
	if (select_direction == INCREASE_DIRECTION)
	{
		while (((u8_routing_data >> (u8_addr + (u8_couter - 1))) & 1) == false)
		{
			u8_couter++;
			if (u8_couter + u8_addr >= 8)
			{
				u8_couter = 0;
				break;
			}
		}
		return u8_addr + u8_couter;
	}
	else if (select_direction == DECREASE_DIRECTION)
	{
		while (((u8_routing_data >> (u8_addr - (u8_couter + 1))) & 1) == false)
		{
			u8_couter++;
			if (u8_couter >= u8_addr)
			{
				u8_couter = 0;
				break;
			}
		}
		return u8_addr - u8_couter;
	}
	return 0;
}

static void v_process_repeat()
{
	if ( stru_address.u8_next_addr > stru_address.u8_low_addr
		&& stru_address.u8_next_addr != 0 )
	{
		// Valve IS Repeat Node
		// Build new array frame control
		uint8_t au8_msg_ctr_new[32];
		v_frame_build(&str_data_ctr, FRAME_TYPE_ID_CTRL,
									stru_address.u8_high_addr, stru_address.u8_next_addr, stru_address.u8_channel,
									au8_msg_ctr_new);
		// Send ctr frame to next addr
		// b_lora_send_data(stru_address.u8_high_addr, stru_address.u8_next_addr, stru_address.u8_channel, au8_msg_ctr_new, au8_msg_ctr_new[0]);
		// Wait alive form next node
		v_timer_reset_couter();
		e_waiting_state = WAITING_NEXT_ACK;
	}
	else 
	{
		// Valve IS End Node
		HAL_Delay(TIME_SPACE_ENDNODE);
		// Add 'str_data_rep.u8_routing_val'
		str_data_rep.u8_routing_val = str_data_ctr.u8_routing_val;
		// Build and send Rep frame
		v_process_report();
	}
}

static void v_process_report()
{
	if (u8_try_send == 0)
	{
		// Add 'u8_battery_val' 'u8_state_val' to rep frame
		bool b_battery_state = (u16_bat_get_vol() >= BAT_LEVER) ? true : false;
		str_data_rep.u8_battery_val = str_data_rep.u8_battery_val | ((uint32_t) b_battery_state << (stru_address.u8_low_addr - 1));
		str_data_rep.u32_state_val   = str_data_rep.u32_state_val   | ((uint32_t) u8_valve_state   << 4*(stru_address.u8_low_addr - 1));
	}
	// Send ctr frame to prev addr
	if ( stru_address.u8_prev_addr < stru_address.u8_low_addr
		&& stru_address.u8_prev_addr != 0)
	{
		// Conveter str_data_rep to array
		uint8_t au8_msg_rep_new[32];
		v_frame_build(&str_data_rep, FRAME_TYPE_ID_REPS, 
									stru_address.u8_high_addr, stru_address.u8_prev_addr, stru_address.u8_channel,
									au8_msg_rep_new);
		// Repeat rep frame to prev addr, valve is NOT Start Node
		// b_lora_send_data(stru_address.u8_high_addr, stru_address.u8_prev_addr, stru_address.u8_channel, au8_msg_rep_new, au8_msg_rep_new[0]);
	}
	else
	{
		// Conveter str_data_rep to array
		uint8_t au8_msg_rep_new[32];
		v_frame_build(&str_data_rep, FRAME_TYPE_ID_REPS, 
									stru_address.u8_high_addr, 0x00, stru_address.u8_channel,
									au8_msg_rep_new);
		// Repeat rep frame to gate, valve IS Start Node
		//b_lora_send_data(stru_address.u8_high_addr, 0x00, stru_address.u8_channel, au8_msg_rep_new, au8_msg_rep_new[0]);
	}
	// Wait alive form prev node
	v_timer_reset_couter();
	e_waiting_state = WAITING_PREV_ACK;
}

static void v_process_ack(uint8_t u8_h_addr_direct, uint8_t u8_l_addr_direct, bool select_direction)
{
	stru_frame_lora_t stru_null;
	uint8_t u8_buf[10];
	if (select_direction == INCREASE_DIRECTION)
	{
		v_frame_build(&stru_null, FRAME_TYPE_PREV_ACK, u8_h_addr_direct, u8_l_addr_direct, stru_address.u8_channel, u8_buf);
	}
	else if (select_direction == DECREASE_DIRECTION)
	{
		v_frame_build(&stru_null, FRAME_TYPE_NEXT_ACK, u8_h_addr_direct, u8_l_addr_direct, stru_address.u8_channel, u8_buf);
	}
	HAL_Delay(TIME_SPACE_ACK);
	//b_lora_send_data(u8_h_addr_direct, u8_l_addr_direct, stru_address.u8_channel, u8_buf, u8_buf[0]);
}
