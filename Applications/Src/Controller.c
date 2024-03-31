//
// Created by Administrator on 2024/3/30.
//
#include "Controller.h"

extern MPU6050DATATYPE Mpu6050_Data;
// 垂直方向PID控制
int Vertical_PID_PD(void) {
  float Bias;
  int balance;
  Bias =
      Mpu6050_Data.Pitch - Mechanical_balance; // 求出平衡的角度中值 和机械相关
  //  printf("Bias : %f, %d\r\n", Bias, 0);
  balance = PID_Balance_Kp * Bias +
            Mpu6050_Data.Gyro_Y * PID_Balance_Kd; // 计算直立PWM

  return balance; // 返回直立PWm
}