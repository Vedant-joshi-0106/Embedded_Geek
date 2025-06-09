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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Display_Rx_Pin GPIO_PIN_1
#define Display_Rx_GPIO_Port GPIOA
#define Voltage_Sensor_Pin GPIO_PIN_5
#define Voltage_Sensor_GPIO_Port GPIOA
#define Current_Sensor_Pin GPIO_PIN_6
#define Current_Sensor_GPIO_Port GPIOA
#define Thermistor_1_Pin GPIO_PIN_4
#define Thermistor_1_GPIO_Port GPIOC
#define Thermistor_2_Pin GPIO_PIN_5
#define Thermistor_2_GPIO_Port GPIOC
#define echo_Pin GPIO_PIN_15
#define echo_GPIO_Port GPIOB
#define trigger_Pin GPIO_PIN_6
#define trigger_GPIO_Port GPIOC
#define MLX_SDA_Pin GPIO_PIN_9
#define MLX_SDA_GPIO_Port GPIOC
#define MLX_SCL_Pin GPIO_PIN_8
#define MLX_SCL_GPIO_Port GPIOA
#define Display_Tx_Pin GPIO_PIN_10
#define Display_Tx_GPIO_Port GPIOC
#define BL_MOSFET_Pin GPIO_PIN_7
#define BL_MOSFET_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
