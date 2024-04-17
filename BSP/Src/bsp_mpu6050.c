#include "bsp_mpu6050.h"

#include "math.h"

static int16_t Mpu6050Addr = 0xD0; // MPU6050的地址
MPU6050DATATYPE mpu;      // MPU6050的数据结构体

/**
 * @brief IIC读数据函数
 *
 * @param DevAddr 从设备地址
 * @param MemAddr 寄存器地址
 * @param oData 数据指针
 * @param DataLen 数据长度
 * @return * int8_t
 */
int8_t Sensor_I2C1_Read(uint16_t DevAddr, uint16_t MemAddr, uint8_t *oData,
                        uint8_t DataLen) {
  return HAL_I2C_Mem_Read(&hi2c1, DevAddr, MemAddr, I2C_MEMADD_SIZE_8BIT, oData,
                          DataLen, 1000);
}

/**
 * @brief IIC写数据函数
 *
 * @param DevAddr 从设备地址
 * @param MemAddr 寄存器地址
 * @param iData 数据指针
 * @param DataLen 数据长度
 * @return int8_t
 */
int8_t Sensor_I2C1_Write(uint16_t DevAddr, uint16_t MemAddr, uint8_t *iData,
                         uint8_t DataLen) {
  return HAL_I2C_Mem_Write(&hi2c1, DevAddr, MemAddr, I2C_MEMADD_SIZE_8BIT,
                           iData, DataLen, 1000);
}

/**
 * @brief 搜索从设备函数
 *
 * @return int16_t 0xD1失败 其他成功
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
 * @brief MPU6050初始化函数
 *
 * @param Addr 从设备地址
 * @return int8_t 0成功 -1失败
 */
int8_t MPU6050_Init_Myself(int16_t Addr) {
  uint8_t check;
  HAL_I2C_Mem_Read(&hi2c1, Addr, WHO_AM_I, 1, &check, 1, 1000);
  if (check == 0x68) // 确认设备用 地址寄存器
  {
    check = 0x00;
    Sensor_I2C1_Write(Addr, PWR_MGMT_1, &check, 1); // 唤醒
    check = 0x07;
    Sensor_I2C1_Write(Addr, SMPLRT_DIV, &check, 1); // 1Khz的速率
    check = 0x00;
    Sensor_I2C1_Write(Addr, ACCEL_CONFIG, &check, 1); // 加速度配置
    check = 0x00;
    Sensor_I2C1_Write(Addr, GYRO_CONFIG, &check, 1); // 陀螺配置
    return 0;
  }
  return -1;
}

/**
 * @brief 读取MPU6050三轴加速度值
 *
 */
void MPU6050_Read_Accel(void) // 读加速度
{
  uint8_t Read_Buf[6];

  // 寄存器依次是加速度X高 - 加速度X低 - 加速度Y高位 - 加速度Y低位 - 加速度Z高位
  // - 加速度度Z低位
  Sensor_I2C1_Read(Mpu6050Addr, ACCEL_XOUT_H, Read_Buf, 6);

  // 原始数据(未来要用与DMP解算)
  mpu.Accel_X = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
  mpu.Accel_Y = (int16_t)(Read_Buf[2] << 8 | Read_Buf[3]);
  mpu.Accel_Z = (int16_t)(Read_Buf[4] << 8 | Read_Buf[5]);
  // 实际数据计算
  mpu.Accel_X = mpu.Accel_X / 16384.0f;
  mpu.Accel_Y = mpu.Accel_Y / 16384.0f;
  mpu.Accel_Z = mpu.Accel_Z / 16384.0f;
}

/**
 * @brief 读取MPU6050三轴偏移角度值
 *
 */
void MPU6050_Read_Gyro(void) // 读角度
{
  uint8_t Read_Buf[6];

  // 寄存器依次是角度X高 - 角度X低 - 角度Y高位 - 角度Y低位 - 角度Z高位 -
  // 角度Z低位
  Sensor_I2C1_Read(Mpu6050Addr, GYRO_XOUT_H, Read_Buf, 6);
  // 原始数据(未来要用与DMP解算)
  mpu.Gyro_X = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
  mpu.Gyro_Y = (int16_t)(Read_Buf[2] << 8 | Read_Buf[3]);
  mpu.Gyro_Z = (int16_t)(Read_Buf[4] << 8 | Read_Buf[5]);
  // 实际数据
  mpu.Gyro_X = mpu.Gyro_X / 131.0f;
  mpu.Gyro_Y = mpu.Gyro_Y / 131.0f;
  mpu.Gyro_Z = mpu.Gyro_Z / 131.0f;
}
/**
 * @brief 读取MPU6050温度值
 *
 */
void MPU6050_Read_Temp(void) // 读温度
{
  uint8_t Read_Buf[2];

  Sensor_I2C1_Read(Mpu6050Addr, TEMP_OUT_H, Read_Buf, 2);
  // 原始数据
  mpu.Temp = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
  // 实际数据
  mpu.Temp = 36.53f + (mpu.Temp / 340.0f);
}

/**
 * @brief 读取数据并计算MPU6050未经滤波的角度值
 *
 */
void MPU6050_Getangle(void) {
  MPU6050_Read_Accel(); // 获取原始加速度
  MPU6050_Read_Gyro();  // 获取原始角速度
  mpu.angleAx = atan2(mpu.Accel_X, mpu.Accel_Z) *
                         180 / 3.141f; // 计算与x轴的夹角
  mpu.angleAy = atan2(mpu.Accel_Y, mpu.Accel_Z) *
                         180 / 3.141f;       // 计算与y轴的夹角
  mpu.gyroGx = mpu.Gyro_X; // 计算角速度
  mpu.gyroGy = mpu.Gyro_Y; // 计算角速度
  mpu.gyroGz = mpu.Gyro_Z; // 计算角速度
}
