/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32g0xx_hal.h"
#include "stm32g0xx_ll_usart.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_dma.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MEM_HOLD_Pin LL_GPIO_PIN_9
#define MEM_HOLD_GPIO_Port GPIOB
#define MEM_WP_Pin LL_GPIO_PIN_14
#define MEM_WP_GPIO_Port GPIOC
#define MEM_CS_Pin LL_GPIO_PIN_15
#define MEM_CS_GPIO_Port GPIOC
#define KEY_1_Pin LL_GPIO_PIN_0
#define KEY_1_GPIO_Port GPIOA
#define KEY_2_Pin LL_GPIO_PIN_1
#define KEY_2_GPIO_Port GPIOA
#define KEY_3_Pin LL_GPIO_PIN_4
#define KEY_3_GPIO_Port GPIOA
#define DISP_SDI_Pin LL_GPIO_PIN_5
#define DISP_SDI_GPIO_Port GPIOA
#define DISP_CS_Pin LL_GPIO_PIN_6
#define DISP_CS_GPIO_Port GPIOA
#define DISP_D_C_Pin LL_GPIO_PIN_7
#define DISP_D_C_GPIO_Port GPIOA
#define DISP_SDO_Pin LL_GPIO_PIN_0
#define DISP_SDO_GPIO_Port GPIOB
#define DISP_SCL_Pin LL_GPIO_PIN_1
#define DISP_SCL_GPIO_Port GPIOB
#define VCTR_Pin LL_GPIO_PIN_2
#define VCTR_GPIO_Port GPIOB
#define DCDC_Pin LL_GPIO_PIN_8
#define DCDC_GPIO_Port GPIOA
#define BF_Pin LL_GPIO_PIN_6
#define BF_GPIO_Port GPIOC
#define TEST_1_Pin LL_GPIO_PIN_11
#define TEST_1_GPIO_Port GPIOA
#define TEST_2_Pin LL_GPIO_PIN_12
#define TEST_2_GPIO_Port GPIOA
#define KEY_4_Pin LL_GPIO_PIN_13
#define KEY_4_GPIO_Port GPIOA
#define KEY_5_Pin LL_GPIO_PIN_14
#define KEY_5_GPIO_Port GPIOA
#define MEM_MISO_Pin LL_GPIO_PIN_6
#define MEM_MISO_GPIO_Port GPIOB
#define MEM_MOSI_Pin LL_GPIO_PIN_7
#define MEM_MOSI_GPIO_Port GPIOB
#define MEM_SCK_Pin LL_GPIO_PIN_8
#define MEM_SCK_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
