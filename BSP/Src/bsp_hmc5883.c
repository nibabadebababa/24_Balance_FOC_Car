#include "bsp_hmc5883.h"
#include <math.h>

struct HMC5883L_Data mag;

//***************************************
// �ϲ������ֽں���
int g85_makeuint16(int msb, int lsb) {
  return ((msb & 0xFF) << 8) | (lsb & 0xFF);
}

// ��ʼ��������HMC5883L
void Init_HMC5883L_HAL(I2C_HandleTypeDef *hi2c1) {
  unsigned char cdata[3] = {0x78, 0x38, 0X00};
  HAL_I2C_Mem_Write(hi2c1, HMC5883L_Addr, HMC5883l_CONFIG_A,
                    I2C_MEMADD_SIZE_8BIT, cdata, 1, 1000);     // 75Hz
  HAL_I2C_Mem_Write(hi2c1, HMC5883L_Addr, HMC5883l_CONFIG_B,
                    I2C_MEMADD_SIZE_8BIT, cdata + 1, 1, 1000); // �������棬
  HAL_I2C_Mem_Write(hi2c1, HMC5883L_Addr, HMC5883l_MODECONFIG,
                    I2C_MEMADD_SIZE_8BIT, cdata + 2, 1,1000);  // ���ò���ģʽ����������ģʽ
}

/**** ��ȡHMC5883L�Ĵų��Ƕ�  */
float read_hmc5883l_HAL(I2C_HandleTypeDef *hi2c1,
                        struct HMC5883L_Data *VL_temp) {
  float fangle;
  int x, y;
  // ������⺯����һ�ζ�6���ֽ���,�����ָ��ĵ���
  HAL_I2C_Mem_Read(hi2c1, HMC5883L_Addr, 0x03, I2C_MEMADD_SIZE_8BIT,
                   VL_temp->vtemp, 6, 1000);

  VL_temp->x_h = g85_makeuint16(VL_temp->vtemp[0], VL_temp->vtemp[1]);
  VL_temp->y_h = g85_makeuint16(VL_temp->vtemp[4], VL_temp->vtemp[5]);
  VL_temp->z_h = g85_makeuint16(VL_temp->vtemp[2], VL_temp->vtemp[3]);

  if (VL_temp->x_h > 0x7fff)
    VL_temp->x_h -= 0xffff; // ����Ԥ������
  if (VL_temp->y_h > 0x7fff)
    VL_temp->y_h -= 0xffff;

  y = VL_temp->y_h;
  x = VL_temp->x_h;
  if (x == 0) {
    fangle = 180;
    if (y > 0)
      fangle = 0;
  }

  if (x < 0)
    fangle = 90 + atan((y + 0.001) / x) *
                      57.3; // ��0.001��Ϊ��ǿ�ƽ��и��������㣬�ҶԾ���Ӱ���С
  if (x > 0)
    fangle = 270 + atan((y + 0.001) / x) * 57.3;
  VL_temp->angle = fangle;

  return fangle;
}

