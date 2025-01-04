/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define BTN3_Pin GPIO_PIN_13
#define BTN3_GPIO_Port GPIOC
#define BTN2_Pin GPIO_PIN_14
#define BTN2_GPIO_Port GPIOC
#define BTN1_Pin GPIO_PIN_15
#define BTN1_GPIO_Port GPIOC
#define SIG3_Pin GPIO_PIN_0
#define SIG3_GPIO_Port GPIOA
#define SIG2_Pin GPIO_PIN_1
#define SIG2_GPIO_Port GPIOA
#define CSP_Pin GPIO_PIN_2
#define CSP_GPIO_Port GPIOA
#define SIG1_Pin GPIO_PIN_3
#define SIG1_GPIO_Port GPIOA
#define CSG_Pin GPIO_PIN_4
#define CSG_GPIO_Port GPIOA
#define SCKM_Pin GPIO_PIN_5
#define SCKM_GPIO_Port GPIOA
#define MISOM_Pin GPIO_PIN_6
#define MISOM_GPIO_Port GPIOA
#define MOSIM_Pin GPIO_PIN_7
#define MOSIM_GPIO_Port GPIOA
#define INTG_Pin GPIO_PIN_0
#define INTG_GPIO_Port GPIOB
#define INTG_EXTI_IRQn EXTI0_IRQn
#define PPS_Pin GPIO_PIN_1
#define PPS_GPIO_Port GPIOB
#define PPS_EXTI_IRQn EXTI1_IRQn
#define DRDYP_Pin GPIO_PIN_2
#define DRDYP_GPIO_Port GPIOB
#define DRDYP_EXTI_IRQn EXTI2_IRQn
#define GTX_Pin GPIO_PIN_10
#define GTX_GPIO_Port GPIOB
#define GRX_Pin GPIO_PIN_11
#define GRX_GPIO_Port GPIOB
#define DC_Pin GPIO_PIN_12
#define DC_GPIO_Port GPIOB
#define SCKD_Pin GPIO_PIN_13
#define SCKD_GPIO_Port GPIOB
#define CSD_Pin GPIO_PIN_14
#define CSD_GPIO_Port GPIOB
#define MOSID_Pin GPIO_PIN_15
#define MOSID_GPIO_Port GPIOB
#define SDAM_Pin GPIO_PIN_8
#define SDAM_GPIO_Port GPIOA
#define SCLM_Pin GPIO_PIN_9
#define SCLM_GPIO_Port GPIOA
#define DRDYM_Pin GPIO_PIN_10
#define DRDYM_GPIO_Port GPIOA
#define DRDYM_EXTI_IRQn EXTI15_10_IRQn
#define SCLE_Pin GPIO_PIN_15
#define SCLE_GPIO_Port GPIOA
#define DRV1_Pin GPIO_PIN_4
#define DRV1_GPIO_Port GPIOB
#define DRV2_Pin GPIO_PIN_5
#define DRV2_GPIO_Port GPIOB
#define REPH_Pin GPIO_PIN_6
#define REPH_GPIO_Port GPIOB
#define SDAE_Pin GPIO_PIN_7
#define SDAE_GPIO_Port GPIOB
#define BTN4_Pin GPIO_PIN_8
#define BTN4_GPIO_Port GPIOB
#define REPL_Pin GPIO_PIN_9
#define REPL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
