/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void mcu_sleep(void);
// uart
bool uart2_available(void);
uint8_t uart2_get_data(uint8_t *pData);
void uart2_transmit(uint8_t *pData, uint16_t Size);
void uart3_transmit(uint8_t *pData, uint16_t Size);
// timer
uint16_t u16_timer_get_couter(void);
void v_timer_reset_couter(void);
// adc
uint16_t u16_adc_get_value(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RF_AUX_Pin GPIO_PIN_0
#define RF_AUX_GPIO_Port GPIOA
#define RF_AUX_EXTI_IRQn EXTI0_IRQn
#define RF_M0_Pin GPIO_PIN_1
#define RF_M0_GPIO_Port GPIOA
#define RF_RX_Pin GPIO_PIN_2
#define RF_RX_GPIO_Port GPIOA
#define RF_TX_Pin GPIO_PIN_3
#define RF_TX_GPIO_Port GPIOA
#define RF_M1_Pin GPIO_PIN_4
#define RF_M1_GPIO_Port GPIOA
#define BATSEN_Pin GPIO_PIN_5
#define BATSEN_GPIO_Port GPIOA
#define EN_RF_Pin GPIO_PIN_6
#define EN_RF_GPIO_Port GPIOA
#define EN_1_Pin GPIO_PIN_7
#define EN_1_GPIO_Port GPIOA
#define EN_2_Pin GPIO_PIN_0
#define EN_2_GPIO_Port GPIOB
#define EN_3_Pin GPIO_PIN_1
#define EN_3_GPIO_Port GPIOB
#define EN_4_Pin GPIO_PIN_2
#define EN_4_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_14
#define LED3_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_15
#define LED4_GPIO_Port GPIOB
#define DIS_1_Pin GPIO_PIN_8
#define DIS_1_GPIO_Port GPIOA
#define DIS_2_Pin GPIO_PIN_9
#define DIS_2_GPIO_Port GPIOA
#define DIS_3_Pin GPIO_PIN_10
#define DIS_3_GPIO_Port GPIOA
#define DIS_4_Pin GPIO_PIN_11
#define DIS_4_GPIO_Port GPIOA
#define EN_SEN_Pin GPIO_PIN_12
#define EN_SEN_GPIO_Port GPIOA
#define SW1_Pin GPIO_PIN_15
#define SW1_GPIO_Port GPIOA
#define SW2_Pin GPIO_PIN_3
#define SW2_GPIO_Port GPIOB
#define SW3_Pin GPIO_PIN_4
#define SW3_GPIO_Port GPIOB
#define SW4_Pin GPIO_PIN_5
#define SW4_GPIO_Port GPIOB
#define SW5_Pin GPIO_PIN_6
#define SW5_GPIO_Port GPIOB
#define SW6_Pin GPIO_PIN_7
#define SW6_GPIO_Port GPIOB
#define SW7_Pin GPIO_PIN_8
#define SW7_GPIO_Port GPIOB
#define STT_Pin GPIO_PIN_9
#define STT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define LEN_BUF_DATA 	32
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
