/**
  ******************************************************************************
  * @file           : build_frame.c
  * @brief          : 
  ******************************************************************************
  */
#include <string.h>
#include "build_frame.h"
#include "setup_address.h"

uint8_t u8_frame_read(uint8_t au8_message_in[], stru_frame_lora_t *pstru_data_out)
{
	// Check lenght
	if (au8_message_in[0] == FRAME_LEN_ACK || au8_message_in[0] == FRAME_LEN_INFO)
	{
		// Do not thing
	}
	else
	{
		return FRAME_ERR_LEN;
	}
	// Check sum
	uint8_t u8_len = au8_message_in[0];
	uint16_t u16_sum_check = 0;
	for(uint8_t i = 1; i <  u8_len - 2; i++)
	{
		u16_sum_check += au8_message_in[i];
	}
	uint16_t u16_sum_data = ((uint16_t)au8_message_in[u8_len - 2] << 8) | (uint16_t)au8_message_in[u8_len - 1];
	if (u16_sum_check != u16_sum_data)
	{
		return FRAME_ERR_SUM;
	}
	// Check address
	if ( au8_message_in[2] != u8_address_get_high()
		|| au8_message_in[3] != u8_address_get_low()
		|| au8_message_in[4] != u8_address_get_chan() )
	{
		return FRAME_ERR_ADDR;
	}
	// Lenght_frame
	pstru_data_out->u8_lenght_frame = au8_message_in[0];
	// NodeTypeID
	pstru_data_out->u8_frame_typeID = au8_message_in[1];
	// Number ID_NODE
	pstru_data_out->u8_high_addr = au8_message_in[2];
	pstru_data_out->u8_low_addr  = au8_message_in[3];
	pstru_data_out->u8_chan_addr = au8_message_in[4];
	switch (au8_message_in[1])
	{
		case FRAME_TYPE_ID_CTRL:
		{
			// Routing valve station
			pstru_data_out->u8_routing_val = au8_message_in[5];
			pstru_data_out->u8_en_state_val = au8_message_in[6];
			// State valve control
			pstru_data_out->u32_state_val = ((uint32_t)au8_message_in[7] << 24) 
																		| ((uint32_t)au8_message_in[8] << 16) 
																		| ((uint32_t)au8_message_in[9] << 8 ) 
																		| ((uint32_t)au8_message_in[10]);
			// SumData
			pstru_data_out->u16_checksum = ((uint16_t)au8_message_in[11] << 8) | ((uint16_t)au8_message_in[12]);
			break;
		}
		case FRAME_TYPE_ID_REPS:
		{
			// Routing valve station
			pstru_data_out->u8_routing_val = au8_message_in[5];
			pstru_data_out->u8_battery_val = au8_message_in[6];
			// State valve control
			pstru_data_out->u32_state_val = ((uint32_t)au8_message_in[7] << 24) 
																		| ((uint32_t)au8_message_in[8] << 16) 
																		| ((uint32_t)au8_message_in[9] << 8 ) 
																		| ((uint32_t)au8_message_in[10]);
			// SumData
			pstru_data_out->u16_checksum = ((uint16_t)au8_message_in[11] << 8) | ((uint16_t)au8_message_in[12]);
			break;
		}
		case FRAME_TYPE_NEXT_ACK:
		{
			// SumData
			pstru_data_out->u16_checksum = ((uint16_t)au8_message_in[5] << 8) | ((uint16_t)au8_message_in[6]);
			break;
		}
		case FRAME_TYPE_PREV_ACK:
		{
			// SumData
			pstru_data_out->u16_checksum = ((uint16_t)au8_message_in[5] << 8) | ((uint16_t)au8_message_in[6]);
			break;
		}
		default: break;
	}
	return au8_message_in[1];
}

void v_frame_build(stru_frame_lora_t *pstru_data_in, uint8_t u8_node_typeID, 
									 uint8_t u8_target_h_addr, uint8_t u8_target_l_addr, uint8_t u8_target_c_addr,
                   uint8_t au8_message_out[])
{
	uint16_t u16_sum_data = 0;
	memset(au8_message_out, 0x00, 16);
	// NodeTypeID
	au8_message_out[1] = u8_node_typeID;
	// Number ID_NODE
	au8_message_out[2] = u8_target_h_addr;
	au8_message_out[3] = u8_target_l_addr;
	au8_message_out[4] = u8_target_c_addr;
	switch (u8_node_typeID)
	{
		case FRAME_TYPE_ID_CTRL:
		{
			// Lenght_frame 
			au8_message_out[0] = FRAME_LEN_INFO;
			// Routing valve
			au8_message_out[5] = pstru_data_in->u8_routing_val;
			au8_message_out[6] = pstru_data_in->u8_en_state_val;
			// State valve
			au8_message_out[7]  = (uint8_t)(pstru_data_in->u32_state_val >> 24) & 0xFF;	
			au8_message_out[8] 	= (uint8_t)(pstru_data_in->u32_state_val >> 16) & 0xFF;			
			au8_message_out[9] 	= (uint8_t)(pstru_data_in->u32_state_val >> 8 ) & 0xFF;			
			au8_message_out[10] = (uint8_t)(pstru_data_in->u32_state_val) & 0xFF;
			break;
		}
		case FRAME_TYPE_ID_REPS:
		{
			// Lenght_frame 
			au8_message_out[0] = FRAME_LEN_INFO;
			// Routing valve
			au8_message_out[5] = pstru_data_in->u8_routing_val;
			au8_message_out[6] = pstru_data_in->u8_battery_val;
			// State valve
			au8_message_out[7]  = (uint8_t)(pstru_data_in->u32_state_val >> 24) & 0xFF;	
			au8_message_out[8] 	= (uint8_t)(pstru_data_in->u32_state_val >> 16) & 0xFF;			
			au8_message_out[9] 	= (uint8_t)(pstru_data_in->u32_state_val >> 8 ) & 0xFF;			
			au8_message_out[10] = (uint8_t)(pstru_data_in->u32_state_val) & 0xFF;
			break;
		}
		case FRAME_TYPE_NEXT_ACK:
		{
			// Lenght_frame 
			au8_message_out[0] = FRAME_LEN_ACK;
			break;
		}
		case FRAME_TYPE_PREV_ACK:
		{
			// Lenght_frame 
			au8_message_out[0] = FRAME_LEN_ACK;
			break;
		}
		default: break;
	}
	// SumData
	for(uint8_t i = 1; i < (au8_message_out[0] - 2); i++)
	{
		u16_sum_data += au8_message_out[i];
	}
	au8_message_out[au8_message_out[0] - 2] = (uint8_t)(u16_sum_data>> 8) & 0xFF;
	au8_message_out[au8_message_out[0] - 1] = (uint8_t)(u16_sum_data) & 0xFF;
}
