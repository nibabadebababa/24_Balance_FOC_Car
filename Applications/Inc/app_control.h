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
  float I;
}PID_TYPE_DEF;

extern PID_TYPE_DEF  Balance ; // ֱ����
extern PID_TYPE_DEF  Velocity; // �ٶȻ�
extern PID_TYPE_DEF  Turn    ; // ת��
extern PID_TYPE_DEF  Location; // λ�û�
extern PID_TYPE_DEF  Dir     ; // ����
extern PID_TYPE_DEF  Follow  ; // ���滷

void PID_Init(void);
void PID_Control_Update(void);
float Balance_PID_Calcu(float target_angle, float current_angle, float gyro);
float Velocity_PID_Calcu(float target_v, float current_v);
float Dir_PID_Calcu(float target_yaw, float current_yaw, float gyro_z);
void Pick_Up_Detect(float velocity, float pitch);
void Falling_Detect(float pitch);
float Turn_PID_Calcu(float RC, float gyro_Z);
void Landing_Detect(void);
float Location_PID_Calcu(float target_s, float current_s, float ax);


#endif



