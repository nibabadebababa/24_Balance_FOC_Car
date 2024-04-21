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
void System_Calibration_Yaw(void);


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_ACTION_Pin GPIO_PIN_8
#define LED_ACTION_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define     PrintChar       printf
#define     UART_BUF_MAX    20
#define     MAG_BUF_LEN     3
#define     TIM2_CNT_MAX    10000

#define     MOTOR0          0
#define     MOTOR1          1

typedef enum {
  ESP32 = 0,
  BLE = 1,
  X3 = 2,
  DAPLINK = 3,
} PRINTF_ENUM_TYPE;

typedef enum{
    ANGLE = 0,      // 角度闭环控制
    FREEDOM = 1,    // 自由转向控制
}TURN_CONTROL_TYPE; // 转弯类型

typedef struct {
  
    PRINTF_ENUM_TYPE print_dev;
    uint8_t Motor_Ready;  // 电机FOC算法磁编码器自检
    uint8_t X3_Ready;     // X3启动launch文件与串口节点
    uint8_t MPU_Ready;    // MPU6050初始化完成
    uint8_t HMC_Ready;    // HMC5883磁力计初始化完成
    
    /* 系统状态 */
    float   Yaw;
    float   Pitch;
    float   Roll;
    float   Gx;
    float   Gy;
    float   Gz;
    float   Ax;
    float   Ay;
    float   Az;
    float   V0;         // 电机0真实速度
    float   V1;         // 电机1真实速度
    float   bat;        // 电池电压
    float   Set_V0;     // 电机0设定速度
    float   Set_V1;     // 电机1设定速度
    float   Yaw_offset; // 磁力计与MPU6050的Yaw角偏差
    uint8_t     low_bat_warning;
    uint8_t     pick_up_flag;
    uint8_t     falling_flag;
    TURN_CONTROL_TYPE     turn_sta; // 转弯类型(角度闭环/自由转向)
    
  
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

extern uint16_t tim2_100ms_cnt;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
