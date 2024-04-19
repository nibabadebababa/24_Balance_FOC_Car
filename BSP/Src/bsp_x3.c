#include "bsp_x3.h"
#include "string.h"
#include "cmsis_os.h"
#include "stdlib.h"

DETECT_TARGET_TYPE target_buf[10];
static uint8_t pointer=0;

void UART3_X3_Rx_Update(void)
{
	if(uart3_rxpointer!=0){
		uint8_t temp = uart3_rxpointer;
		vTaskDelay(1);
		if(temp == uart3_rxpointer)
			UART3_X3_Proc();
	}
}
// 0x00,\n
// 0xFF,id,x,y,h,w
void UART3_X3_Proc(void)
{
	uint8_t flag=0;
	if(uart3_rxdata[0] == 0xFF){
		sscanf((char*)uart3_rxdata, "%d,%d,%d,%d,%d,%d\n",&flag, &target_buf[pointer].id ,\
								&target_buf[pointer].x_offset, &target_buf[pointer].y_offset ,\
								&target_buf[pointer].height, &target_buf[pointer].width);
	}
	

	memset(uart3_rxdata, 0, UART_BUF_MAX);
	uart3_rxpointer = 0;
}
