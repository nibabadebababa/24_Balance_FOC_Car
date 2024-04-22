#include "bsp_x3.h"
#include "string.h"
#include "cmsis_os.h"
#include "stdlib.h"
#include "stdio.h"


#define     TARGET_BUF_LEN      3

//DETECT_TARGET_TYPE target_buf[TARGET_BUF_LEN];

static int id_buffer[TARGET_BUF_LEN];
static int x_buffer[TARGET_BUF_LEN];
static int y_buffer[TARGET_BUF_LEN];
static int h_buffer[TARGET_BUF_LEN];
static int w_buffer[TARGET_BUF_LEN];

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
// 0xFE,ID, XH, XL, YH, YL, HeightH, HeightL, WideH, WideL 0xEF
void UART3_X3_Proc(void)
{
    //HAL_UART_Transmit(&huart6, uart3_rxdata, uart3_rxpointer,10);
    //int data1,data2,data3,data4,data5;
    sscanf((char*)uart3_rxdata, "%d,%d,%d,%d,%d",   &id_buffer[pointer], \
                                                    &x_buffer[pointer], \
                                                    &y_buffer[pointer], \
                                                    &h_buffer[pointer], \
                                                    &w_buffer[pointer]);
    pointer++;
    if(pointer >= TARGET_BUF_LEN){
        sys.yolo.id = id_buffer[TARGET_BUF_LEN-1];
        sys.yolo.x_offset = Median_int_Filter(x_buffer, TARGET_BUF_LEN);
        sys.yolo.y_offset = Median_int_Filter(y_buffer, TARGET_BUF_LEN);
        sys.yolo.height   = Median_int_Filter(h_buffer, TARGET_BUF_LEN);
        sys.yolo.width    = Median_int_Filter(w_buffer, TARGET_BUF_LEN);
//        printf("%d,%d,%d,%d,%d\n",  sys.yolo.id ,\
//                                    sys.yolo.x_offset, \
//                                    sys.yolo.y_offset ,\
//                                    sys.yolo.height, \
//                                    sys.yolo.width);
        pointer = 0;
    }

	memset(uart3_rxdata, 0, UART_BUF_MAX);
	uart3_rxpointer = 0;
}

int Median_int_Filter(int* buffer, uint8_t len)
{
    uint8_t i,j;
    int temp;
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
