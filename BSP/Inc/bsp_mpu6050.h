#ifndef __BSP_MPU6050_H
#define __BSP_MPU6050_H

#include "main.h"

#define SMPLRT_DIV 		0x19   	// 采样率分频，典型值：0x07(125Hz) */
#define CONFIG 			0x1A    // 低通滤波频率，典型值：0x06(5Hz) */
#define GYRO_CONFIG 	0x1B  	// 陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s) */
#define ACCEL_CONFIG 	0x1C 	// 加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz) */

// 存储最近的X轴、Y轴、Z轴加速度感应器的测量值 */
#define ACCEL_XOUT_H 	0x3B 	// 加速度值,X轴高8位寄存器
#define ACCEL_XOUT_L 	0x3C 	// 加速度值,X轴低8位寄存器
#define ACCEL_YOUT_H 	0x3D 	// 加速度值,Y轴高8位寄存器
#define ACCEL_YOUT_L 	0x3E 	// 加速度值,Y轴低8位寄存器
#define ACCEL_ZOUT_H 	0x3F 	// 加速度值,Z轴高8位寄存器
#define ACCEL_ZOUT_L 	0x40 	// 加速度值,Z轴低8位寄存器

// 存储的最近温度传感器的测量值 */
#define TEMP_OUT_H 		0x41 	// 温度值高八位寄存器
#define TEMP_OUT_L 		0x42 	// 温度值低8位寄存器

// 存储最近的X轴、Y轴、Z轴陀螺仪感应器的测量值 */
#define GYRO_XOUT_H 	0x43 	// 陀螺仪值,X轴高8位寄存器
#define GYRO_XOUT_L 	0x44 	// 陀螺仪值,X轴低8位寄存器
#define GYRO_YOUT_H 	0x45 	// 陀螺仪值,Y轴高8位寄存器
#define GYRO_YOUT_L 	0x46 	// 陀螺仪值,Y轴低8位寄存器
#define GYRO_ZOUT_H 	0x47 	// 陀螺仪值,Z轴高8位寄存器
#define GYRO_ZOUT_L 	0x48 	// 陀螺仪值,Z轴低8位寄存器

#define PWR_MGMT_1 		0x6B   	// 电源管理，典型值：0x00(正常启用) */
#define WHO_AM_I 		0x75    // IIC地址寄存器(默认数值0x68，只读) */
#define MPU6050_ADDR 	0xD0 	// MPU6050手册上的地址，这里也可以使用serch函数去搜索

typedef struct
{
    // 加速度
    float Accel_X;
    float Accel_Y;
    float Accel_Z;
    // 角速度
    float Gyro_X;
    float Gyro_Y;
    float Gyro_Z;

    // 实际角度
    float angleAx;
    float angleAy;
    float angleAz; // 在小车项目中不会用到

    // 实际角速度
    float gyroGy;
    float gyroGx;
    float gyroGz;
    // 欧拉角
    float Pitch;
    float Roll;
    float Yaw; // 在小车项目中不会用到
    //  温度
    float Temp;
} MPU6050DATATYPE;

extern MPU6050DATATYPE mpu;
extern I2C_HandleTypeDef hi2c1;

int16_t Sensor_I2C1_Serch(void);
int8_t MPU6050_Init_Myself(int16_t Addr);

int8_t Sensor_I2C1_ReadOneByte(uint16_t DevAddr, uint16_t MemAddr, uint8_t *oData);
int8_t Sensor_I2C1_WriteOneByte(uint16_t DevAddr, uint16_t MemAddr, uint8_t *iData);

void MPU6050_Read_Accel(void);
void MPU6050_Read_Gyro(void);
void MPU6050_Read_Temp(void);
void MPU6050_Getangle(void);






#endif

