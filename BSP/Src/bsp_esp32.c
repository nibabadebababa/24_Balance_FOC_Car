#include "bsp_esp32.h"
#include "stdio.h"
#include "string.h"
#include "cmsis_os.h"

#define 	TORQUE_MAX		12

void UART6_DAPLink_Rx_Update(void)
{
	if(uart6_rxpointer !=0){
		uint8_t temp = uart6_rxpointer;
		vTaskDelay(1);
		if(temp == uart6_rxpointer)
			UART6_DAPLink_Proc();
	}
}

void UART2_ESP32_Rx_Update(void)
{
	if(uart2_rxpointer !=0){
		uint8_t temp = uart2_rxpointer;
		vTaskDelay(1);
		if(temp == uart2_rxpointer)
			UART2_ESP32_Proc();
	}	
}

void UART6_DAPLink_Proc(void)
{
	HAL_UART_Transmit(&huart2, uart6_rxdata, uart6_rxpointer, 10);
	
	uart6_rxpointer = 0;
	memset(uart6_rxdata, 0, UART_BUF_MAX);
}

void UART2_ESP32_Proc(void)
{
	HAL_UART_Transmit(&huart6, uart2_rxdata, uart2_rxpointer, 10);
	sscanf((char*)uart2_rxdata, "%f,%f\n",&sys.V1,&sys.V2);
	uart2_rxpointer = 0;
	memset(uart2_rxdata, 0, UART_BUF_MAX);	
}


void Set_Motor_Torque(uint8_t motor, float torque)
{
	char message[10] = {'\0'};
	if(torque > 0)
		torque = (torque>TORQUE_MAX)?(TORQUE_MAX):(torque);
	else
		torque = (torque<-TORQUE_MAX)?(-TORQUE_MAX):(torque);

	if(motor == MOTOR0)
		sprintf(message, (char*)"A%.2f\n",torque);
	else if(motor == MOTOR1)
		sprintf(message, (char*)"B%.2f\n",torque);
	
	HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 1);
}

