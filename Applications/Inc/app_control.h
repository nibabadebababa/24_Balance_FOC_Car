#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H

#include "PID.h"
#include "bsp_mpu6050.h"
#include "main.h"
#include "stdio.h"

#define Mechanical_balance 1.0f // 机械平衡角度
#define PID_Balance_Kp 27.0f    // 直立控制比例系数
#define PID_Balance_Kd 0.6f     // 直立控制微分系数

int Vertical_PID_PD(void);

#endif
