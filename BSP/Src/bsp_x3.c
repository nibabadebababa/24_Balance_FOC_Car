#include "bsp_x3.h"
#include "stdio.h"
#include "string.h"

void UART3_X3_Proc(void) {

  memset(uart3_rxdata, 0, UART_BUF_MAX);
  uart3_rxpointer = 0;
}
