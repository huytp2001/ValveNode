/**
  ******************************************************************************
  * @file           : flash_mem.h
  * @brief          : 
  ******************************************************************************
  */
#ifndef FLASH_MEM_H
#define FLASH_MEM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "stdbool.h"

uint32_t u32_flash_write(uint8_t *data, uint16_t size_byte);
uint8_t u8_flash_read(uint8_t index);

#ifdef __cplusplus
}
#endif

#endif
