#include "bsp_bluetooth.h"
#include "string.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "app_bluetooth.h"


BT_PARA_TYPE_DEF bt;


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



