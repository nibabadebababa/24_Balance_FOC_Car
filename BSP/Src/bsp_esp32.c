#include "bsp_esp32.h"
#include "stdio.h"

#define 	TORQUE_MAX		40


void Set_Motor_Torque(uint8_t motor, int16_t torque)
{
	sys.print_dev = ESP32;
	torque = (torque>TORQUE_MAX)?(TORQUE_MAX):(torque);
	if(motor == MOTOR0){
		printf("A%d",torque);
	}
	else if(motor == MOTOR1){
		printf("B%d",torque);
	}
}

