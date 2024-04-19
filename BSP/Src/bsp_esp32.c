#include "bsp_esp32.h"
#include "stdio.h"
#include "string.h"
#include "cmsis_os.h"

#define 	TORQUE_MAX		120
#define 	V_BUF_LEN		5

float Velocity_Buf0[V_BUF_LEN];
float Velocity_Buf1[V_BUF_LEN];
uint8_t buf_pointer=0;

void UART6_DAPLink_Rx_Update(void)
{
	if(uart6_rxpointer !=0){
		uint8_t temp = uart6_rxpointer;
		vTaskDelay(1);
		if(temp == uart6_rxpointer) UART6_DAPLink_Proc();
	}
}

void UART2_ESP32_Rx_Update(void)
{
	if(uart2_rxpointer !=0){
		uint8_t temp = uart2_rxpointer;
		vTaskDelay(1);
		if(temp == uart2_rxpointer) UART2_ESP32_Proc();
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
  if(sys.Motor_Ready == 0){
    if( strcmp((char*)uart2_rxdata, "OK") == 0 ){
        HAL_GPIO_WritePin(LED_ACTION_GPIO_Port, LED_ACTION_Pin, GPIO_PIN_RESET);
        sys.Motor_Ready = 1;
    }
  } 
  else{
    sscanf((char*)uart2_rxdata, "%f,%f\n",&Velocity_Buf0[buf_pointer],&Velocity_Buf1[buf_pointer]);
    buf_pointer++;
    buf_pointer%=V_BUF_LEN;
    sys.V0 = Velocity_Filter(Velocity_Buf0);
    sys.V1 = Velocity_Filter(Velocity_Buf1);
  }

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
		sprintf(message, (char*)"A%.2f\n",torque/10.0f);
	else if(motor == MOTOR1){
		sprintf(message, (char*)"B%.2f\n",-torque/10.0f);
	}
		
	
	HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 1);
}

float Velocity_Filter(float* buffer)
{
	float sum=0;
	uint8_t i;
	for(i=0;i<V_BUF_LEN;i++)
		sum += buffer[i];
	
	return (sum/V_BUF_LEN);
}

