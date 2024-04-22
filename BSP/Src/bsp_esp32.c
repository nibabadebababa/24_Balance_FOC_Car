#include "bsp_esp32.h"
#include "stdio.h"
#include "string.h"
#include "cmsis_os.h"
#include "stdlib.h"

#define 	TORQUE_MAX		140
#define 	V_BUF_LEN		5
#define     S_BUF_LEN       5
float Velocity_Buf0[V_BUF_LEN];
float Velocity_Buf1[V_BUF_LEN];
float S_Buf0[S_BUF_LEN];
float S_Buf1[S_BUF_LEN];

uint8_t buf_pointer=0;
uint8_t sp = 0;

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
    Velocity_Buf0[buf_pointer] = (float)((short)(( ((short)uart2_rxdata[0]) <<8) | uart2_rxdata[1]))/10.0f;
    Velocity_Buf1[buf_pointer++] = (float)((short)(( ((short)uart2_rxdata[2]) <<8) | uart2_rxdata[3]))/10.0f;
    S_Buf0[sp] = (float)((short)(( ((short)uart2_rxdata[4]) <<8) | uart2_rxdata[5]))/10.0f;
    S_Buf1[sp++] = (float)((short)(( ((short)uart2_rxdata[6]) <<8) | uart2_rxdata[7]))/10.0f;

    if(buf_pointer>=V_BUF_LEN){
        sys.V0 = Median_Filter(Velocity_Buf0, V_BUF_LEN);
        sys.V1 = Median_Filter(Velocity_Buf1, V_BUF_LEN);
        buf_pointer = 0;
    }
    if(sp >= S_BUF_LEN){
        sys.S0 = Median_Filter(S_Buf0, S_BUF_LEN);
        sys.S1 = Median_Filter(S_Buf1, S_BUF_LEN);        
        sp = 0;
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
		sprintf(message, (char*)"A%.2f\n",torque/10.0f);  // 放大十倍后 计算原数值
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

float Average_Filter(float* buffer, uint8_t len)
{
    uint8_t i;
    float sum = 0;
    for(i=0;i<len;i++)
        sum += buffer[i];
    
    return (sum/len);
}


float Median_Filter(float* buffer, uint8_t len)
{
    uint8_t i,j;
    float temp;
    for(j = 0; j< len - 1;j++){
        for(i=0;i<len - j - 1;i++){
            if(buffer[i] > buffer[i+1]){
                temp = buffer[i];
                buffer[i] = buffer[i+1];
                buffer[i+1] = temp;
            }
            
        }   
    }
    if(len%2==0){
        temp = (buffer[len/2]+buffer[len/2-1])/2.0f;
    } else{
        temp = buffer[(len-1)/2];
    }
    
    return temp;
}
