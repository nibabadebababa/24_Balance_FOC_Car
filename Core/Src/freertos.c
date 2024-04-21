/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_control.h"
#include "app_bluetooth.h"
#include "bsp_battry.h"
#include "bsp_dmp.h"
#include "bsp_esp32.h"
#include "bsp_hmc5883.h"
#include "bsp_mpu6050.h"
#include "bsp_bluetooth.h"
#include "bsp_x3.h"

#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define VELOCITY_DEBUG    // 不等待电机自检完成
#define YAW_ONLY_USE_MPU    // 只使用MPU6050获取Yaw角
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  BaseType_t preTick = xTaskGetTickCount();
  
    uint8_t cnt = 0;
  while(sys.Motor_Ready==0){
      printf("Waiting for Motor Self-Detect...\n");
    vTaskDelay(100);
  }
  System_Calibration_Yaw();
  
  for (;;) {
    System_Get_Pose();
      if(cnt){
          System_Get_Yaw();
          cnt=0;
      }
      printf("%.1f\n",sys.Pitch);
    //Falling_Detect(sys.Pitch);
    Pick_Up_Detect(sys.V0-sys.V1,sys.Pitch);
    //printf("%.1f,%.1f,%.1f,%d\n",sys.Yaw, mag.angle, mpu.Yaw, tim2_100ms_cnt);
    //printf("%.1f,%.1f,%.1f\n",sys.V0-sys.V1, sys.Pitch, sys.Yaw);
    UART2_ESP32_Rx_Update();
      
      if(sys.falling_flag || sys.pick_up_flag){
          Set_Motor_Torque(MOTOR0, 0);
          Set_Motor_Torque(MOTOR1, 0);
      }
      else{
          PID_Control_Update();
      }
    
    cnt++;
    vTaskDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the myTask02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  uint8_t i;
  /* Infinite loop */
  for (;;) {
    if(sys.falling_flag){
          /* 闪烁一次：检测到倒地 */
          HAL_GPIO_WritePin(LED_ACTION_GPIO_Port, LED_ACTION_Pin, GPIO_PIN_RESET);
          vTaskDelay(200);  
          HAL_GPIO_WritePin(LED_ACTION_GPIO_Port, LED_ACTION_Pin, GPIO_PIN_SET); 
          vTaskDelay(1000);           
      }
      else if(sys.pick_up_flag){
          /* 闪烁三次：检测到拿起 */
          for(i=0;i<3;i++){
              HAL_GPIO_WritePin(LED_ACTION_GPIO_Port, LED_ACTION_Pin, GPIO_PIN_RESET);
              vTaskDelay(200);  
              HAL_GPIO_WritePin(LED_ACTION_GPIO_Port, LED_ACTION_Pin, GPIO_PIN_SET);
              vTaskDelay(200);  
          }
          vTaskDelay(1000);           
      }
      else if(sys.bat<11.5){
          /* 熄灭：检测到电池电压过低 */
          HAL_GPIO_WritePin(LED_ACTION_GPIO_Port, LED_ACTION_Pin, GPIO_PIN_SET);
      }
      else{
          /* 一直闪烁：正常*/
          HAL_GPIO_TogglePin(LED_ACTION_GPIO_Port, LED_ACTION_Pin);
          vTaskDelay(100);
      }
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
 * @brief Function implementing the myTask03 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  //  int speed = 0;
  /* Infinite loop */
  for (;;) {
    UART1_BLE_Rx_Update();
    UART2_ESP32_Rx_Update();
    UART3_X3_Rx_Update();
    UART6_DAPLink_Rx_Update();
    Bluetooth_Cmd_Pro();
      System_Get_Battry();
  }
  /* USER CODE END StartTask03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void System_Init(void)
{
    sys.bat = 0;
    sys.print_dev = DAPLINK;
    sys.V0 = 0;
    sys.V1 = 0;
    sys.low_bat_warning = 0;  // 低压警告
    sys.Set_V0 = 0;
    sys.Set_V1 = 0;
    sys.X3_Ready = 0;
    #ifdef VELOCITY_DEBUG
    sys.Motor_Ready = 1;
    #else
    sys.Motor_Ready = 0;
    #endif
    
    sys.MPU_Ready = 0;
    sys.HMC_Ready = 0;
    sys.pick_up_flag = 0;       // 拿起检测标志位
    sys.falling_flag = 0;       // 倒地检测标志位
    sys.turn_sta = ANGLE;
    
    /* 蓝牙初始状态 */
    bt.cmd = Self_Ctrl;
    bt.rx_flag = 0;
}

void System_Get_Pose(void)
{
  MPU6050_Get_Pose();
  MPU6050_Read_Gyro();
//  MPU6050_Read_Accel();
//  sys.Ax = mpu.Accel_X;
//  sys.Ay = mpu.Accel_Y;
//  sys.Az = mpu.Accel_Z;
  sys.Gx = mpu.Gyro_X;
  sys.Gy = mpu.Gyro_Y;
  sys.Gz = mpu.Gyro_Z;
  sys.Pitch = mpu.Pitch;
  sys.Roll = mpu.Roll;
}

void System_Get_Yaw(void)
{
#ifndef YAW_ONLY_USE_MPU
    static float mag_buffer[MAG_BUF_LEN] = {0};
    static float mpu_buffer[MAG_BUF_LEN] = {0};
    static uint8_t buf_pointer = 0;
    float a = 0.9f;
    
    HMC5883_Get_Yaw(&hi2c1, &mag);
    mag_buffer[buf_pointer] = mag.angle;
    mpu_buffer[buf_pointer] = mpu.Yaw;
    buf_pointer++;
    if(buf_pointer>= MAG_BUF_LEN){
        float mag_yaw = Median_Filter(mag_buffer, MAG_BUF_LEN);
        float mpu_yaw = Average_Filter(mpu_buffer, MAG_BUF_LEN);
        sys.Yaw = a*(mpu_yaw) + (1-a)*(mag_yaw + sys.Yaw_offset);
        printf("%.1f,%.1f,%.1f,%.1f\n",sys.Yaw, mpu_yaw, mag.angle-180, mag_yaw+sys.Yaw_offset);
        buf_pointer = 0;
    }
#else
    static float mpu_buffer[MAG_BUF_LEN] = {0};
    static uint8_t buf_pointer = 0;
    mpu_buffer[buf_pointer++] = mpu.Yaw;
    if(buf_pointer>= MAG_BUF_LEN){
        sys.Yaw = Average_Filter(mpu_buffer, MAG_BUF_LEN);
        buf_pointer = 0;
    }
    
#endif
}

void System_Get_Battry(void)
{
  Battry_GetVoltage();
}

/* 初始校准Yaw角 */
void System_Calibration_Yaw(void)
{
    float mag_buffer[10] = {0};
    uint8_t cnt = 0;
    if(sys.MPU_Ready==1 && sys.HMC_Ready==1){
        do{
            HMC5883_Get_Yaw(&hi2c1, &mag);
            mag_buffer[cnt++] = mag.angle;
            if(cnt>=10){
                MPU6050_Get_Pose();
                HAL_Delay(1);
                float mag_yaw = Median_Filter(mag_buffer, 10);
                sys.Yaw_offset = mpu.Yaw - mag_yaw - 180.0f;
                printf("%.1f,%.1f,%.1f\n",mpu.Yaw, mag_yaw,sys.Yaw_offset);
                break;
            } else
                HAL_Delay(2);
        }while(cnt<10);
    } 
    else 
        printf("Error\n");
}

/* USER CODE END Application */

