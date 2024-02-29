/**
  ******************************************************************************
  * @file           : build_frame.h
  * @brief          : 
  ******************************************************************************
  */

#ifndef BUILD_FRAME_H
#define BUILD_FRAME_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define FRAME_LEN_INFO			13
#define FRAME_LEN_ACK				7

#define FRAME_TYPE_ID_CTRL 	0x43 //'C'
#define FRAME_TYPE_ID_REPS 	0x52 //'R'
#define FRAME_TYPE_NEXT_ACK 0x3E //'>'
#define FRAME_TYPE_PREV_ACK 0x3C //'<'

#define FRAME_ERR_LEN				0x4C //'L'
#define FRAME_ERR_SUM				0x53 //'S'
#define FRAME_ERR_ADDR			0x41 //'A'

typedef struct
{
	uint8_t u8_lenght_frame;
	uint8_t u8_frame_typeID;
	uint8_t u8_high_addr;
	uint8_t u8_low_addr;
	uint8_t u8_chan_addr;
	uint8_t u8_routing_val;
	uint8_t u8_en_state_val;
	uint8_t u8_battery_val;
	uint16_t u16_checksum;
	uint32_t u32_state_val;
} stru_frame_lora_t;

uint8_t u8_frame_read(uint8_t au8_message_in[], stru_frame_lora_t *pstru_data_out);
void v_frame_build(stru_frame_lora_t *pstru_data_in, uint8_t u8_node_typeID, 
									 uint8_t u8_target_h_addr, uint8_t u8_target_l_addr, uint8_t u8_target_c_addr,
                   uint8_t au8_message_out[]);

#ifdef __cplusplus
}
#endif

#endif
