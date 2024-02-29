/**
  ******************************************************************************
  * @file           : setup_address.h
  * @brief          : 
  ******************************************************************************
  */

#ifndef SETUP_ADDR_H
#define SETUP_ADDR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


#define ADDR_CHAN_868MHZ 0x06  		// 868Mhz
#define ADDR_CHAN_915MHZ 0x35 	  // 915Mhz

void u8_address_setup(void);
uint8_t u8_address_get_high(void);
uint8_t u8_address_get_low(void);
uint8_t u8_address_get_chan(void);

#ifdef __cplusplus
}
#endif

#endif
