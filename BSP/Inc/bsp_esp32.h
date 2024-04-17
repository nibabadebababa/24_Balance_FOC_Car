#ifndef __BSP_ESP32_H
#define __BSP_ESP32_H

#include "main.h"

void UART6_Receive_Update(void);
void UART2_Receive_Update(void);
void UART6_Debug_Proc(void);
void UART2_ESP32_Proc(void);

void Set_Motor_Torque(uint8_t motor, float torque);



#endif

