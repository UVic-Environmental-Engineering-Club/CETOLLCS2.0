/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Roll_DIR_Pin GPIO_PIN_5
#define Roll_DIR_GPIO_Port GPIOE
#define Roll_PWM_Pin GPIO_PIN_6
#define Roll_PWM_GPIO_Port GPIOE
#define Roll_POT_Pin GPIO_PIN_4
#define Roll_POT_GPIO_Port GPIOF
#define GRN_OB_LED1_Pin GPIO_PIN_6
#define GRN_OB_LED1_GPIO_Port GPIOF
#define GRN_OB_LED2_Pin GPIO_PIN_7
#define GRN_OB_LED2_GPIO_Port GPIOF
#define RED_OB_LED1_Pin GPIO_PIN_8
#define RED_OB_LED1_GPIO_Port GPIOF
#define RED_OB_LED2_Pin GPIO_PIN_9
#define RED_OB_LED2_GPIO_Port GPIOF
#define Ballast_SW_Pin GPIO_PIN_0
#define Ballast_SW_GPIO_Port GPIOC
#define Roll_SW_Pin GPIO_PIN_1
#define Roll_SW_GPIO_Port GPIOC
#define Pitch_UART_TX_Pin GPIO_PIN_2
#define Pitch_UART_TX_GPIO_Port GPIOA
#define Pitch_UART_RX_Pin GPIO_PIN_3
#define Pitch_UART_RX_GPIO_Port GPIOA
#define Pitch_STEP_Pin GPIO_PIN_5
#define Pitch_STEP_GPIO_Port GPIOA
#define Pitch_DIR_Pin GPIO_PIN_6
#define Pitch_DIR_GPIO_Port GPIOA
#define Pitch_EN_Pin GPIO_PIN_7
#define Pitch_EN_GPIO_Port GPIOA
#define Current_Sense_Pin GPIO_PIN_0
#define Current_Sense_GPIO_Port GPIOB
#define VBatt_Sense_Pin GPIO_PIN_1
#define VBatt_Sense_GPIO_Port GPIOB
#define PEN_SHDN_Pin GPIO_PIN_11
#define PEN_SHDN_GPIO_Port GPIOF
#define HECTO_DCD_Pin GPIO_PIN_14
#define HECTO_DCD_GPIO_Port GPIOE
#define HECTO_NET_AVA_Pin GPIO_PIN_15
#define HECTO_NET_AVA_GPIO_Port GPIOE
#define HECTO_UART_TX_Pin GPIO_PIN_10
#define HECTO_UART_TX_GPIO_Port GPIOB
#define HECTO_UART_RX_Pin GPIO_PIN_11
#define HECTO_UART_RX_GPIO_Port GPIOB
#define HECTO_DTR_Pin GPIO_PIN_12
#define HECTO_DTR_GPIO_Port GPIOB
#define HECTO_UART_CTS_Pin GPIO_PIN_13
#define HECTO_UART_CTS_GPIO_Port GPIOB
#define HECTO_UART_RTS_Pin GPIO_PIN_14
#define HECTO_UART_RTS_GPIO_Port GPIOB
#define HECTO_DSR_Pin GPIO_PIN_15
#define HECTO_DSR_GPIO_Port GPIOB
#define HECTO_RI_Pin GPIO_PIN_8
#define HECTO_RI_GPIO_Port GPIOD
#define GPIO_Spare_PD12_Pin GPIO_PIN_12
#define GPIO_Spare_PD12_GPIO_Port GPIOD
#define GPIO_Spare_PD13_Pin GPIO_PIN_13
#define GPIO_Spare_PD13_GPIO_Port GPIOD
#define GPIO_Spare_PD14_Pin GPIO_PIN_14
#define GPIO_Spare_PD14_GPIO_Port GPIOD
#define GPIO_Spare_PD15_Pin GPIO_PIN_15
#define GPIO_Spare_PD15_GPIO_Port GPIOD
#define RPI_UART_TX_Pin GPIO_PIN_6
#define RPI_UART_TX_GPIO_Port GPIOC
#define RPI_UART_RX_Pin GPIO_PIN_7
#define RPI_UART_RX_GPIO_Port GPIOC
#define GPIO_Spare_PC11_Pin GPIO_PIN_11
#define GPIO_Spare_PC11_GPIO_Port GPIOC
#define GPIO_Spare_PC12_Pin GPIO_PIN_12
#define GPIO_Spare_PC12_GPIO_Port GPIOC
#define Draw_Wire_Set_Zero_Pin GPIO_PIN_2
#define Draw_Wire_Set_Zero_GPIO_Port GPIOD
#define CAN_STBY_Pin GPIO_PIN_3
#define CAN_STBY_GPIO_Port GPIOD
#define Pitch_SW_Pin GPIO_PIN_6
#define Pitch_SW_GPIO_Port GPIOD
#define Pump_PWM_Pin GPIO_PIN_5
#define Pump_PWM_GPIO_Port GPIOB
#define Pump_DIR_Pin GPIO_PIN_6
#define Pump_DIR_GPIO_Port GPIOB
#define Pump_EN_Pin GPIO_PIN_7
#define Pump_EN_GPIO_Port GPIOB
#define Roll_Hall_Pin GPIO_PIN_0
#define Roll_Hall_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
