#ifndef __BSP_HMC5883_H
#define __BSP_HMC5883_H

#include "main.h"

#define ITG3205_Addr  0x68 // 定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改
#define HMC5883L_Addr 0x3C // 磁场传感器器件地址

// 定义HMC5883L配置寄存器地址
#define HMC5883l_CONFIG_A 	0x00
#define HMC5883l_CONFIG_B 	0x01
#define HMC5883l_MODECONFIG 0x02

struct HMC5883L_Data{
  unsigned char vtemp[12];
  int x_h; // x方向磁强
  int y_h; // y方向磁强
  int z_h; // z方向磁强
  float angle;
};

extern struct HMC5883L_Data mag;	// Magnetometer

//***************************************
int g85_makeuint16(int msb, int lsb);
void Init_HMC5883L_HAL(I2C_HandleTypeDef *hi2c1);

/**** 读取磁场角度  */
float read_hmc5883l_HAL(I2C_HandleTypeDef *hi2c1,
                        struct HMC5883L_Data *VL_temp);


#endif

