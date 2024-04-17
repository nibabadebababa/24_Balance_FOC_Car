#include "bsp_battry.h"

#define  QUEUE_LEN	10

extern ADC_HandleTypeDef hadc1;

uint16_t bat_queue[QUEUE_LEN] = {0};
uint8_t queue_pointer = 0;
float Voltage_Calibration = 0.15f;

/*更新电池电压
 *ADC时钟频率 = 21MHz
 *采样与转换需要的周期 = 12.5+480=492.5 个时钟周期
 *总时间 = 492.5/21 = 23.4 us
*/ 
void Battry_GetVoltage(void)
{
	uint16_t temp;
	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1, 2) == HAL_OK){
		bat_queue[queue_pointer++] = HAL_ADC_GetValue(&hadc1);
		queue_pointer = queue_pointer%QUEUE_LEN;

		temp = Get_Queue_Average(bat_queue, QUEUE_LEN);
		sys.bat = temp*3.3f*6.0f/4096.0f + Voltage_Calibration;
	}
	HAL_ADC_Stop(&hadc1);
}

uint32_t Get_Queue_Average(uint16_t* queue, uint8_t len)
{
	uint32_t sum = 0;
	uint8_t i;
	
	for(i=0;i<len;i++)
	{
		sum += queue[i];
	}
	sum = sum/len;
	return sum;
}


