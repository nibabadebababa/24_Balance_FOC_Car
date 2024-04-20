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
  
  for (;;) {
    System_Get_Pose();
      if(cnt){
          System_Get_Yaw();
          cnt=0;
      }
    Falling_Detect(sys.Pitch);
    Pick_Up_Detect(sys.V0-sys.V1,sys.Pitch);
    //printf("%.1f,%.1f,%.1f,%d\n",sys.Yaw, mag.angle, mpu.Yaw, tim2_100ms_cnt);
      printf("%.1f,%.1f\n",sys.V0-sys.V1, sys.Pitch);
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
    //System_Get_Battry();
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
  
  /* Infinite loop */
  for (;;) {
      if(sys.Motor_Ready){
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
    
    sys.pick_up_flag = 0;       // 拿起检测标志位
    sys.falling_flag = 0;       // 倒地检测标志位
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
        sys.Yaw = a*(mpu_yaw+243) + (1-a)*mag_yaw;
        buf_pointer = 0;
    }
}

void System_Get_Battry(void)
{
  Battry_GetVoltage();
}

/* USER CODE END Application */

