#ifndef __BSP_MPU6050_H
#define __BSP_MPU6050_H

#include "main.h"

#define SMPLRT_DIV 		0x19   	// �����ʷ�Ƶ������ֵ��0x07(125Hz) */
#define CONFIG 			0x1A    // ��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz) */
#define GYRO_CONFIG 	0x1B  	// �������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s) */
#define ACCEL_CONFIG 	0x1C 	// ���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz) */

// �洢�����X�ᡢY�ᡢZ����ٶȸ�Ӧ���Ĳ���ֵ */
#define ACCEL_XOUT_H 	0x3B 	// ���ٶ�ֵ,X���8λ�Ĵ���
#define ACCEL_XOUT_L 	0x3C 	// ���ٶ�ֵ,X���8λ�Ĵ���
#define ACCEL_YOUT_H 	0x3D 	// ���ٶ�ֵ,Y���8λ�Ĵ���
#define ACCEL_YOUT_L 	0x3E 	// ���ٶ�ֵ,Y���8λ�Ĵ���
#define ACCEL_ZOUT_H 	0x3F 	// ���ٶ�ֵ,Z���8λ�Ĵ���
#define ACCEL_ZOUT_L 	0x40 	// ���ٶ�ֵ,Z���8λ�Ĵ���

// �洢������¶ȴ������Ĳ���ֵ */
#define TEMP_OUT_H 		0x41 	// �¶�ֵ�߰�λ�Ĵ���
#define TEMP_OUT_L 		0x42 	// �¶�ֵ��8λ�Ĵ���

// �洢�����X�ᡢY�ᡢZ�������Ǹ�Ӧ���Ĳ���ֵ */
#define GYRO_XOUT_H 	0x43 	// ������ֵ,X���8λ�Ĵ���
#define GYRO_XOUT_L 	0x44 	// ������ֵ,X���8λ�Ĵ���
#define GYRO_YOUT_H 	0x45 	// ������ֵ,Y���8λ�Ĵ���
#define GYRO_YOUT_L 	0x46 	// ������ֵ,Y���8λ�Ĵ���
#define GYRO_ZOUT_H 	0x47 	// ������ֵ,Z���8λ�Ĵ���
#define GYRO_ZOUT_L 	0x48 	// ������ֵ,Z���8λ�Ĵ���

#define PWR_MGMT_1 		0x6B   	// ��Դ��������ֵ��0x00(��������) */
#define WHO_AM_I 		0x75    // IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��) */
#define MPU6050_ADDR 	0xD0 	// MPU6050�ֲ��ϵĵ�ַ������Ҳ����ʹ��serch����ȥ����

typedef struct
{
    // ���ٶ�
    float Accel_X;
    float Accel_Y;
    float Accel_Z;
    // ���ٶ�
    float Gyro_X;
    float Gyro_Y;
    float Gyro_Z;

    // ʵ�ʽǶ�
    float angleAx;
    float angleAy;
    float angleAz; // ��С����Ŀ�в����õ�

    // ʵ�ʽ��ٶ�
    float gyroGy;
    float gyroGx;
    float gyroGz;
    // ŷ����
    float Pitch;
    float Roll;
    float Yaw; // ��С����Ŀ�в����õ�
    //  �¶�
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

