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

void System_Init(void);
void System_Get_Pose(void);
void System_Get_Yaw(void);
void System_Get_Battry(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_ACTION_Pin GPIO_PIN_8
#define LED_ACTION_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define PrintChar printf
#define UART_BUF_MAX 20
#define MOTOR0 0
#define MOTOR1 1

typedef enum {
  ESP32 = 0,
  BLE = 1,
  X3 = 2,
  DAPLINK = 3,
} PRINTF_ENUM_TYPE;

typedef struct {
  
  PRINTF_ENUM_TYPE print_dev;
  uint8_t Motor_Ready;  // ���FOC�㷨�ű������Լ�
  uint8_t X3_Ready;     // X3����launch�ļ��봮�ڽڵ�
  
  /* ϵͳ״̬ */
  float   Yaw;
  float   Pitch;
  float   Roll;
  float   Gx;
  float   Gy;
  float   Gz;
  float   Ax;
  float   Ay;
  float   Az;
  float   V0;     // ���0��ʵ�ٶ�
  float   V1;     // ���1��ʵ�ٶ�
  float   bat;    // ��ص�ѹ
  float   Set_V0; // ���0�趨�ٶ�
  float   Set_V1; // ���1�趨�ٶ�
  uint8_t low_bat_warning;
}SYSTEM_TYPE_DEF;

extern SYSTEM_TYPE_DEF sys;

extern uint8_t uart1_rxdat;
extern uint8_t uart1_rxdata[];
extern uint8_t uart1_rxpointer;

extern uint8_t uart2_rxdat;
extern uint8_t uart2_rxdata[];
extern uint8_t uart2_rxpointer;

extern uint8_t uart3_rxdat;
extern uint8_t uart3_rxdata[];
extern uint8_t uart3_rxpointer;

extern uint8_t uart6_rxdat;
extern uint8_t uart6_rxdata[];
extern uint8_t uart6_rxpointer;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
