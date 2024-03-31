#include "bsp_motor.h"

extern TIM_HandleTypeDef htim1;

/*
 *
 */
void motor_set_speed(uint8_t motorID, int16_t speed) {
  // 输入限幅
  if (speed > MOTOR_MAX_SPEED) {
    speed = MOTOR_MAX_SPEED;
  } else if (speed < -MOTOR_MAX_SPEED) {
    speed = -MOTOR_MAX_SPEED;
  }
  switch (motorID) {
  case MOTOR0: // 右电机
    if (speed >= 0) {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
    } else {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -speed);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    }
    break;
  case MOTOR1: // 左电机
    if (speed >= 0) {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
    } else {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -speed);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
    }
    break;
  }
}
