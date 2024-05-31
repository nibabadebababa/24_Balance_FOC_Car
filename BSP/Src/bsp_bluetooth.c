#include "bsp_bluetooth.h"
#include "string.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "app_bluetooth.h"
#include "app_control.h"


BT_PARA_TYPE_DEF bt;
extern PID_TYPE_DEF FollowLoc;

void UART1_BLE_Rx_Update(void)
{
	if(uart1_rxpointer!=0){
		uint8_t temp = uart1_rxpointer;
		vTaskDelay(1);
		if(temp == uart1_rxpointer) UART1_BLE_Proc();
	}
}

/*蓝牙接收处理函数*/
void UART1_BLE_Proc(void)
{
	//HAL_UART_Transmit(&huart6,uart1_rxdata, uart1_rxpointer,10);
	if( strcmp((char*)uart1_rxdata, "A") == 0){
        bt.cmd = Forward;  
        bt.rx_flag = 1;
        UART1_BLE_Print("Forward\n");
        
    } else if ( strcmp((char*)uart1_rxdata, "B") == 0){
        bt.cmd = Backward;
        bt.rx_flag = 1;
        UART1_BLE_Print("Backward\n");
        
    } else if( strcmp((char*)uart1_rxdata, "C") == 0 ){
        bt.cmd = Left_Rot;
        bt.rx_flag = 1;
        UART1_BLE_Print("Left Rotation\n");
        
    } else if( strcmp((char*)uart1_rxdata, "D") == 0) {
        bt.cmd = Right_Rot;
        bt.rx_flag = 1;
        UART1_BLE_Print("Right Rotation\n");
        
    } else if( strcmp((char*)uart1_rxdata, "F") == 0){
        bt.cmd = Left_Rot_45;
        bt.rx_flag = 1;
        UART1_BLE_Print("Left Rotation 45\n");
        
    } else if( strcmp((char*)uart1_rxdata, "G") == 0 ){
        bt.cmd = Right_Rot_45;
        bt.rx_flag = 1;
        UART1_BLE_Print("Right Rotation 45\n");       
    } else if( strcmp((char*)uart1_rxdata, "H") == 0 ){
        bt.cmd = Location_Fix;
        bt.rx_flag = 1;
        UART1_BLE_Print("Location Fixed Mode\n");       
    } else if( strcmp((char*)uart1_rxdata, "I") == 0 ){
        bt.cmd = Angle_Fix;
        bt.rx_flag = 1;
        UART1_BLE_Print("Angle Fixed Mode\n");       
    } else if( strcmp((char*)uart1_rxdata, "J") == 0 ){
        bt.cmd = Location_YOLO;
        bt.rx_flag = 1;
        UART1_BLE_Print("Location Follow With YOLO\n");       
    } else if( strcmp((char*)uart1_rxdata, "K") == 0 ){
        bt.cmd = Angle_YOLO;
        bt.rx_flag = 1;
        UART1_BLE_Print("Angle Follow With YOLO\n");       
    } else if( strcmp((char*)uart1_rxdata, "L") == 0 ){
        char text[25];
        FollowLoc.target += 50;
        FollowLoc.target = ((int)FollowLoc.target)%700;
        sprintf(text, (char*)"YOLO Target Size=%.1f\n", FollowLoc.target);
        UART1_BLE_Print(text);       
    } else if( strcmp((char*)uart1_rxdata, "M") == 0 ){
        char text[25];
        FollowLoc.target -= 50;
        if(FollowLoc.target < 100) FollowLoc.target = 100;
        sprintf(text, (char*)"YOLO Target Size=%.1f\n", FollowLoc.target);
        UART1_BLE_Print(text);  
    }
    else {
        bt.cmd = Self_Ctrl;
        bt.rx_flag = 1;
        UART1_BLE_Print("Auto Control\n");
    }        
	/*清零接收缓冲区与指针*/
	uart1_rxpointer = 0;
	memset(uart1_rxdata, 0, UART_BUF_MAX);
}

void UART1_BLE_Print(const char* message)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen((char*)message), 10);
}



