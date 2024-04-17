#ifndef __BSP_HMC5883_H
#define __BSP_HMC5883_H

#include "main.h"

#define ITG3205_Addr  0x68 // ����������IIC�����еĴӵ�ַ,����ALT  ADDRESS��ַ���Ų�ͬ�޸�
#define HMC5883L_Addr 0x3C // �ų�������������ַ

// ����HMC5883L���üĴ�����ַ
#define HMC5883l_CONFIG_A 	0x00
#define HMC5883l_CONFIG_B 	0x01
#define HMC5883l_MODECONFIG 0x02

struct HMC5883L_Data{
  unsigned char vtemp[12];
  int x_h; // x�����ǿ
  int y_h; // y�����ǿ
  int z_h; // z�����ǿ
  float angle;
};

extern struct HMC5883L_Data mag;	// Magnetometer

//***************************************
int g85_makeuint16(int msb, int lsb);
void Init_HMC5883L_HAL(I2C_HandleTypeDef *hi2c1);

/**** ��ȡ�ų��Ƕ�  */
float read_hmc5883l_HAL(I2C_HandleTypeDef *hi2c1,
                        struct HMC5883L_Data *VL_temp);


#endif

