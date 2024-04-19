#ifndef __BSP_ESP32_H
#define __BSP_ESP32_H

#include "main.h"

void UART6_DAPLink_Rx_Update(void);
void UART2_ESP32_Rx_Update(void);
void UART6_DAPLink_Proc(void);
void UART2_ESP32_Proc(void);

void Set_Motor_Torque(uint8_t motor, float torque);
float Velocity_Filter(float* buffer);


#endif

