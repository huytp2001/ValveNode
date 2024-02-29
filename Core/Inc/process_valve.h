/**
  ******************************************************************************
  * @file           : process_val.h
  * @brief          : 
  ******************************************************************************
  */

#ifndef PROCESS_VAL_H
#define PROCESS_VAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
	WAITING_CONTROL = 0,
	WAITING_NEXT_ACK,
	WAITING_REPLY,
	WAITING_PREV_ACK,
} e_valve_waiting_t;

typedef struct {
	uint8_t u8_high_addr;
	uint8_t u8_low_addr;
	uint8_t u8_channel;
	uint8_t u8_prev_addr;
	uint8_t u8_next_addr;
} stru_valve_addr_t;

#define VALVE_STEP_IN_SEC 	10   // 10steps in 1s
#define TIME_SPACE_ACK			100  // ms
#define TIME_SPACE_ENDNODE	500  // ms
#define TRY_SEND_MAX_NUM    3

void v_process_loop(void);

#ifdef __cplusplus
}
#endif

#endif
