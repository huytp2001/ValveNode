/**
  ******************************************************************************
  * @file           : battery.h
  * @brief          : 
  ******************************************************************************
  */
#ifndef BAT_H
#define BAT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#define BAT_MILLIVOL 	3300
#define BAT_FACTOR 		2
#define BAT_LEVER			3000

uint16_t u16_bat_get_vol(void);

#ifdef __cplusplus
}
#endif

#endif
