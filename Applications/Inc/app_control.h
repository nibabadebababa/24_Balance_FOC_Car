#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H

#include "PID.h"
#include "bsp_mpu6050.h"
#include "main.h"
#include "stdio.h"


typedef struct{
  float Kp;
  float Ki;
  float Kd;
}PID_TYPE_DEF;


float Balance_PID_Update(void);
float Velocity_PID_Update(float target_v);


#endif
