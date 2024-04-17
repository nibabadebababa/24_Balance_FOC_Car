#include "bsp_mpu6050.h"

#include "math.h"

static int16_t Mpu6050Addr = 0xD0; // MPU6050�ĵ�ַ
MPU6050DATATYPE mpu;      // MPU6050�����ݽṹ��

/**
 * @brief IIC�����ݺ���
 *
 * @param DevAddr ���豸��ַ
 * @param MemAddr �Ĵ�����ַ
 * @param oData ����ָ��
 * @param DataLen ���ݳ���
 * @return * int8_t
 */
int8_t Sensor_I2C1_Read(uint16_t DevAddr, uint16_t MemAddr, uint8_t *oData,
                        uint8_t DataLen) {
  return HAL_I2C_Mem_Read(&hi2c1, DevAddr, MemAddr, I2C_MEMADD_SIZE_8BIT, oData,
                          DataLen, 1000);
}

/**
 * @brief IICд���ݺ���
 *
 * @param DevAddr ���豸��ַ
 * @param MemAddr �Ĵ�����ַ
 * @param iData ����ָ��
 * @param DataLen ���ݳ���
 * @return int8_t
 */
int8_t Sensor_I2C1_Write(uint16_t DevAddr, uint16_t MemAddr, uint8_t *iData,
                         uint8_t DataLen) {
  return HAL_I2C_Mem_Write(&hi2c1, DevAddr, MemAddr, I2C_MEMADD_SIZE_8BIT,
                           iData, DataLen, 1000);
}

/**
 * @brief �������豸����
 *
 * @return int16_t 0xD1ʧ�� �����ɹ�
 */
int16_t Sensor_I2C1_Serch(void) // IIC Sensor search function
{
  for (uint8_t i = 1; i < 255; i++) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, i, 1, 1000) == HAL_OK) {
      Mpu6050Addr = i;
      return i;
    }
  }
  return 0xD1;
}

/**
 * @brief MPU6050��ʼ������
 *
 * @param Addr ���豸��ַ
 * @return int8_t 0�ɹ� -1ʧ��
 */
int8_t MPU6050_Init_Myself(int16_t Addr) {
  uint8_t check;
  HAL_I2C_Mem_Read(&hi2c1, Addr, WHO_AM_I, 1, &check, 1, 1000);
  if (check == 0x68) // ȷ���豸�� ��ַ�Ĵ���
  {
    check = 0x00;
    Sensor_I2C1_Write(Addr, PWR_MGMT_1, &check, 1); // ����
    check = 0x07;
    Sensor_I2C1_Write(Addr, SMPLRT_DIV, &check, 1); // 1Khz������
    check = 0x00;
    Sensor_I2C1_Write(Addr, ACCEL_CONFIG, &check, 1); // ���ٶ�����
    check = 0x00;
    Sensor_I2C1_Write(Addr, GYRO_CONFIG, &check, 1); // ��������
    return 0;
  }
  return -1;
}

/**
 * @brief ��ȡMPU6050������ٶ�ֵ
 *
 */
void MPU6050_Read_Accel(void) // �����ٶ�
{
  uint8_t Read_Buf[6];

  // �Ĵ��������Ǽ��ٶ�X�� - ���ٶ�X�� - ���ٶ�Y��λ - ���ٶ�Y��λ - ���ٶ�Z��λ
  // - ���ٶȶ�Z��λ
  Sensor_I2C1_Read(Mpu6050Addr, ACCEL_XOUT_H, Read_Buf, 6);

  // ԭʼ����(δ��Ҫ����DMP����)
  mpu.Accel_X = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
  mpu.Accel_Y = (int16_t)(Read_Buf[2] << 8 | Read_Buf[3]);
  mpu.Accel_Z = (int16_t)(Read_Buf[4] << 8 | Read_Buf[5]);
  // ʵ�����ݼ���
  mpu.Accel_X = mpu.Accel_X / 16384.0f;
  mpu.Accel_Y = mpu.Accel_Y / 16384.0f;
  mpu.Accel_Z = mpu.Accel_Z / 16384.0f;
}

/**
 * @brief ��ȡMPU6050����ƫ�ƽǶ�ֵ
 *
 */
void MPU6050_Read_Gyro(void) // ���Ƕ�
{
  uint8_t Read_Buf[6];

  // �Ĵ��������ǽǶ�X�� - �Ƕ�X�� - �Ƕ�Y��λ - �Ƕ�Y��λ - �Ƕ�Z��λ -
  // �Ƕ�Z��λ
  Sensor_I2C1_Read(Mpu6050Addr, GYRO_XOUT_H, Read_Buf, 6);
  // ԭʼ����(δ��Ҫ����DMP����)
  mpu.Gyro_X = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
  mpu.Gyro_Y = (int16_t)(Read_Buf[2] << 8 | Read_Buf[3]);
  mpu.Gyro_Z = (int16_t)(Read_Buf[4] << 8 | Read_Buf[5]);
  // ʵ������
  mpu.Gyro_X = mpu.Gyro_X / 131.0f;
  mpu.Gyro_Y = mpu.Gyro_Y / 131.0f;
  mpu.Gyro_Z = mpu.Gyro_Z / 131.0f;
}
/**
 * @brief ��ȡMPU6050�¶�ֵ
 *
 */
void MPU6050_Read_Temp(void) // ���¶�
{
  uint8_t Read_Buf[2];

  Sensor_I2C1_Read(Mpu6050Addr, TEMP_OUT_H, Read_Buf, 2);
  // ԭʼ����
  mpu.Temp = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
  // ʵ������
  mpu.Temp = 36.53f + (mpu.Temp / 340.0f);
}

/**
 * @brief ��ȡ���ݲ�����MPU6050δ���˲��ĽǶ�ֵ
 *
 */
void MPU6050_Getangle(void) {
  MPU6050_Read_Accel(); // ��ȡԭʼ���ٶ�
  MPU6050_Read_Gyro();  // ��ȡԭʼ���ٶ�
  mpu.angleAx = atan2(mpu.Accel_X, mpu.Accel_Z) *
                         180 / 3.141f; // ������x��ļн�
  mpu.angleAy = atan2(mpu.Accel_Y, mpu.Accel_Z) *
                         180 / 3.141f;       // ������y��ļн�
  mpu.gyroGx = mpu.Gyro_X; // ������ٶ�
  mpu.gyroGy = mpu.Gyro_Y; // ������ٶ�
  mpu.gyroGz = mpu.Gyro_Z; // ������ٶ�
}
