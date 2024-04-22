#ifndef __BSP_X3_H
#define __BSP_X3_H

#include "main.h"




extern DETECT_TARGET_TYPE target_buf[];
int Median_int_Filter(int* buffer, uint8_t len);
void UART3_X3_Rx_Update(void);
void UART3_X3_Proc(void);


#endif

