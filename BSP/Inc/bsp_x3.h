#ifndef __BSP_X3_H
#define __BSP_X3_H

#include "main.h"


typedef struct{
	uint16_t id;
	uint16_t x_offset;
	uint16_t y_offset;
	uint16_t height;
	uint16_t width;
}DETECT_TARGET_TYPE;

extern DETECT_TARGET_TYPE target_buf[];

void UART3_X3_Rx_Update(void);
void UART3_X3_Proc(void);


#endif

