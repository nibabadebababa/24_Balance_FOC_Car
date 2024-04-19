#include "app_control.h"

#define   OUTPUT_MAX  100
#define   MED_ANGLE   4

PID_TYPE_DEF  Balance   = {2, 0, 0.5};  // Kp=2, Ki, Kd
PID_TYPE_DEF  Velocity  = {1, 0.005, 0};  // Kp, Ki, Kd
PID_TYPE_DEF  Location  = {1, 0, 0};  // Kp, Ki, Kd
PID_TYPE_DEF  Angle     = {1, 0, 0};  // Kp, Ki, Kd
PID_TYPE_DEF  Follow    = {1, 0, 0};  // Kp, Ki, Kd


float Balance_PID_Update(void)
{
  float PID_out = 0;
  
  PID_out = Balance.Kp*(MED_ANGLE - sys.Pitch) + Balance.Kd*(sys.Gy - 0);
  
  return PID_out;
}

float Velocity_PID_Update(float target_v)
{
  static float Shift = 0,Intergral_MAX=1000;
  float Error;
  float PID_out = 0;
  
  Error = target_v - (sys.V0+sys.V1)/2.0f;
  
  Shift += Error;
  if(Shift > Intergral_MAX) Shift = Intergral_MAX;
  else if(Shift < -Intergral_MAX) Shift = -Intergral_MAX;
  
  PID_out = Velocity.Kp*Error + Velocity.Ki*Shift;
  
  return PID_out;
}


//// 垂直方向PID控制
//int Vertical_PID_PD(void) {
//  float Bias;
//  int balance;
//  Bias = mpu.Pitch - Mechanical_balance; // 求出平衡的角度中值 和机械相关
//  //  printf("Bias : %f, %d\r\n", Bias, 0);
//  balance = PID_Balance_Kp * Bias + mpu.Gyro_Y * PID_Balance_Kd; // 计算直立PWM

//  return balance; // 返回直立PWm
//}
