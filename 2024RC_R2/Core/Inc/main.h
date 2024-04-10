/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx.h"
#include <stdio.h>
#include <stdarg.h> 
#include "stdbool.h"
#include "string.h"
#include "user_can_init.h"
#include "can.h"
#include "gpio.h"
#include "motor_drive.h"
#include "chassis.h"
#include "pid.h"
#include "math.h"
#include "usart.h"
#include "config.h"
#include "communication.h"
#include "vesc_can.h"
#include "rm_motor.h"
#include "air_joy.h"
#include "FSM.h"
#include "body_controllers.h"
#include "MIT.h"
#include "tim.h"
#include "pid.h"
#include "haoying_u8.h"
#include "action.h"
#include "move.h"
#include "path.h"

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
#define AIN1_Pin GPIO_PIN_0
#define AIN1_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_1
#define AIN2_GPIO_Port GPIOA
#define BIN2_Pin GPIO_PIN_8
#define BIN2_GPIO_Port GPIOC
#define BIN1_Pin GPIO_PIN_9
#define BIN1_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
