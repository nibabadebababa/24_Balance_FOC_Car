#ifndef __BSP_BLUETOOTH_H
#define __BSP_BLUETOOTH_H

#include "main.h"


typedef enum{
    Forward = 'A',
    Backward = 'B',
    Left_Rot = 'C',
    Right_Rot = 'D',
    Self_Ctrl = 'E',
    Left_Rot_45 = 'F',
    Right_Rot_45 = 'G',
}BT_CMD_TYPE_DEF;

typedef struct{
    BT_CMD_TYPE_DEF cmd;
    uint8_t         rx_flag;
    
}BT_PARA_TYPE_DEF;

extern BT_PARA_TYPE_DEF bt;

void UART1_BLE_Rx_Update(void);
void UART1_BLE_Proc(void);
void UART1_BLE_Print(const char* message);

#endif

