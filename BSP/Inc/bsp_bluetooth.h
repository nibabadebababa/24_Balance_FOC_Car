#ifndef __BSP_BLUETOOTH_H
#define __BSP_BLUETOOTH_H

#include "main.h"


typedef enum{
    Forward = 'A',      // 连续前进
    Backward = 'B',     // 连续后退
    Left_Rot = 'C',     // 连续左转
    Right_Rot = 'D',    // 连续右转
    Self_Ctrl = 'E',    // 自稳
    Left_Rot_45 = 'F',  // 左转45度
    Right_Rot_45 = 'G', // 右转45度
    Location_Fix = 'H', // 位置闭环模式
    Angle_Fix = 'I',    // 角度闭环模式
    Location_YOLO = 'J',// 位置跟随模式（YOLO）
    Angle_YOLO = 'K',   // 角度跟随模式（YOLO）
    YOLO_TAR_ADD = 'L', // YOLO目标大小增大
    YOLO_TAR_SUB = 'M', // YOLO目标大小减小
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

