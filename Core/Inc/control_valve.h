/**
  ******************************************************************************
  * @file           : control_valve.h
  * @brief          : 
  ******************************************************************************
  */

#ifndef CTRL_VALVE_H
#define CTRL_VALVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

void v_ctrl_coil_action(void);
void v_ctrl_set(uint8_t u8_state);

#ifdef __cplusplus
}
#endif

#endif
