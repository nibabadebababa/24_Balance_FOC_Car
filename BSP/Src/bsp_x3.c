#include "bsp_x3.h"
#include "string.h"
#include "cmsis_os.h"

void UART3_X3_Rx_Update(void)
{
	if(uart3_rxpointer!=0){
		uint8_t temp = uart3_rxpointer;
		vTaskDelay(1);
		if(temp == uart3_rxpointer)
			UART3_X3_Proc();
	}
}


void UART3_X3_Proc(void)
{

  memset(uart3_rxdata, 0, UART_BUF_MAX);
  uart3_rxpointer = 0;
}
