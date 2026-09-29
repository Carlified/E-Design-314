/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
enum load_options{
	on,
	off,
	toggle,
};
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern SPI_HandleTypeDef hspi1;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void toggle_load(enum load_options option);
void stats(void);
void poll_keypad(void);
void add_keypress(char);
void add_unit_digit(int8_t digit);
void add_units(uint32_t units);
void process(uint16_t * startPos);
void activateAntenna();
void deactivateAntenna();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DEBUG_BTN_Pin GPIO_PIN_13
#define DEBUG_BTN_GPIO_Port GPIOC
#define DEBUG_BTN_EXTI_IRQn EXTI4_15_IRQn
#define RFID_CS_Pin GPIO_PIN_0
#define RFID_CS_GPIO_Port GPIOF
#define ROW4_Pin GPIO_PIN_0
#define ROW4_GPIO_Port GPIOC
#define COL3_Pin GPIO_PIN_1
#define COL3_GPIO_Port GPIOC
#define Voltage_Pin GPIO_PIN_0
#define Voltage_GPIO_Port GPIOA
#define Current_Pin GPIO_PIN_1
#define Current_GPIO_Port GPIOA
#define RFID_CK_Pin GPIO_PIN_5
#define RFID_CK_GPIO_Port GPIOA
#define RFID_MISO_Pin GPIO_PIN_6
#define RFID_MISO_GPIO_Port GPIOA
#define RFID_MOSI_Pin GPIO_PIN_7
#define RFID_MOSI_GPIO_Port GPIOA
#define D4_Pin GPIO_PIN_4
#define D4_GPIO_Port GPIOC
#define PUSHBTN_Pin GPIO_PIN_5
#define PUSHBTN_GPIO_Port GPIOC
#define PUSHBTN_EXTI_IRQn EXTI4_15_IRQn
#define COL2_Pin GPIO_PIN_0
#define COL2_GPIO_Port GPIOB
#define SD_SCK_Pin GPIO_PIN_13
#define SD_SCK_GPIO_Port GPIOB
#define SD_MISO_Pin GPIO_PIN_14
#define SD_MISO_GPIO_Port GPIOB
#define SD_MOSI_Pin GPIO_PIN_15
#define SD_MOSI_GPIO_Port GPIOB
#define D2_Pin GPIO_PIN_8
#define D2_GPIO_Port GPIOA
#define COL1_Pin GPIO_PIN_9
#define COL1_GPIO_Port GPIOA
#define CS__SD_Pin GPIO_PIN_6
#define CS__SD_GPIO_Port GPIOC
#define ROW1_Pin GPIO_PIN_7
#define ROW1_GPIO_Port GPIOC
#define D3_Pin GPIO_PIN_10
#define D3_GPIO_Port GPIOA
#define ROW3_Pin GPIO_PIN_11
#define ROW3_GPIO_Port GPIOA
#define ROW2_Pin GPIO_PIN_12
#define ROW2_GPIO_Port GPIOA
#define RFID_RST_Pin GPIO_PIN_15
#define RFID_RST_GPIO_Port GPIOA
#define D5_Pin GPIO_PIN_3
#define D5_GPIO_Port GPIOB
#define RFID_IRQ_Pin GPIO_PIN_7
#define RFID_IRQ_GPIO_Port GPIOB
#define RFID_IRQ_EXTI_IRQn EXTI4_15_IRQn
#define OLED_SCL_Pin GPIO_PIN_8
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_9
#define OLED_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define SD_SPI_HANDLE hspi2

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
