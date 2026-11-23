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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
//创建所有任务全局枚举
typedef enum {
  SELECT_TASK=0U,//选择任务
  TASK_CONFIG=1U,  //任务配置
  TASK_RUN=2U,     //任务运行
  TASK_END=3U       //任务结束
} all_task_state_t;

/**
  * @brief  故障上报：记下触发者、点亮两颗用户灯、用寄存器直发打一行
  * @param  tag 触发点名字（编译期字符串常量，例如 "IMU_Init"）
  * @note   和 Error_Handler 的分工：
  *           Error_Handler —— HAL/CubeMX 约定的统一入口，不知道具体是谁触发，
  *                            所以它调 Fault_Report("hal/mx")；
  *           各初始化步骤   —— 自己调 Fault_Report("那一步的名字")，
  *                            这样串口上直接能读出"死在哪一步"。
  */
void Fault_Report(const char *tag);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define key1_Pin GPIO_PIN_2
#define key1_GPIO_Port GPIOE
#define key2_Pin GPIO_PIN_3
#define key2_GPIO_Port GPIOE
#define key3_Pin GPIO_PIN_4
#define key3_GPIO_Port GPIOE
#define CIN1_Pin GPIO_PIN_5
#define CIN1_GPIO_Port GPIOE
#define CIN2_Pin GPIO_PIN_6
#define CIN2_GPIO_Port GPIOE
#define FENGMING_Pin GPIO_PIN_1
#define FENGMING_GPIO_Port GPIOC
#define ENC_L_A_Pin GPIO_PIN_0
#define ENC_L_A_GPIO_Port GPIOA
#define ENC_L_B_Pin GPIO_PIN_1
#define ENC_L_B_GPIO_Port GPIOA
#define ENC_R_B_Pin GPIO_PIN_7
#define ENC_R_B_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_9
#define AIN1_GPIO_Port GPIOE
#define AIN2_Pin GPIO_PIN_11
#define AIN2_GPIO_Port GPIOE
#define BIN1_Pin GPIO_PIN_13
#define BIN1_GPIO_Port GPIOE
#define BIN2_Pin GPIO_PIN_14
#define BIN2_GPIO_Port GPIOE
#define DIN1_Pin GPIO_PIN_14
#define DIN1_GPIO_Port GPIOB
#define DIN2_Pin GPIO_PIN_15
#define DIN2_GPIO_Port GPIOB
#define ENC_R2_B_Pin GPIO_PIN_12
#define ENC_R2_B_GPIO_Port GPIOD
#define ENC_R2_A_Pin GPIO_PIN_13
#define ENC_R2_A_GPIO_Port GPIOD
#define ENC_L2_A_Pin GPIO_PIN_6
#define ENC_L2_A_GPIO_Port GPIOC
#define ENC_L2_B_Pin GPIO_PIN_7
#define ENC_L2_B_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
