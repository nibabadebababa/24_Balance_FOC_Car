#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H

#include "bsp_mpu6050.h"
#include "main.h"
#include "stdio.h"


typedef struct{
  float Kp;
  float Ki;
  float Kd;
  float target;
  float out;
  float I_MAX;
  float outMAX;
  float outMIN;
}PID_TYPE_DEF;


float Balance_PID_Calcu(float target_angle, float current_angle, float gyro);
float Velocity_PID_Calcu(float target_v, float current_v);


#endif
