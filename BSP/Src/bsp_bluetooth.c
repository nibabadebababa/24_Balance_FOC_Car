#include "bsp_bluetooth.h"
#include "string.h"
#include "cmsis_os.h"
#include "stdio.h"

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
	HAL_UART_Transmit(&huart6,uart1_rxdata, uart1_rxpointer,10);
	
	/*清零接收缓冲区与指针*/
	uart1_rxpointer = 0;
	memset(uart1_rxdata, 0, UART_BUF_MAX);
}

void UART1_BLE_Print(const char* message)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen((char*)message), 10);
}



