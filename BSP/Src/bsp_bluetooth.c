#include "bsp_bluetooth.h"
#include "string.h"
#include "cmsis_os.h"

void UART1_BLE_Rx_Update(void)
{
	if(uart1_rxpointer!=0){
		uint8_t temp = uart1_rxpointer;
		vTaskDelay(1);
		if(temp == uart1_rxpointer)
			UART1_BLE_Proc();
	}
}

void UART1_BLE_Proc(void)
{
	
	
	uart1_rxpointer = 0;
	memset(uart1_rxdata, 0, UART_BUF_MAX);
}



