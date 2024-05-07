#ifndef __BSP_BLUETOOTH_H
#define __BSP_BLUETOOTH_H

#include "main.h"


typedef enum{
    Forward = 'A',      // ����ǰ��
    Backward = 'B',     // ��������
    Left_Rot = 'C',     // ������ת
    Right_Rot = 'D',    // ������ת
    Self_Ctrl = 'E',    // ����
    Left_Rot_45 = 'F',  // ��ת45��
    Right_Rot_45 = 'G', // ��ת45��
    Location_Fix = 'H', // λ�ñջ�ģʽ
    Angle_Fix = 'I',    // �Ƕȱջ�ģʽ
    Location_YOLO = 'J',// λ�ø���ģʽ��YOLO��
    Angle_YOLO = 'K',   // �Ƕȸ���ģʽ��YOLO��
    YOLO_TAR_ADD = 'L', // YOLOĿ���С����
    YOLO_TAR_SUB = 'M', // YOLOĿ���С��С
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

