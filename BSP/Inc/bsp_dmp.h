#ifndef __DMP_H__
#define __DMP_H__

#include "main.h"

#define q30 1073741824.0f

extern float Pitch, Roll, Yaw;
extern short gyro[3], accel[3];
// extern float gyro[0], gyro[1], gyro[2], accel[0],accel[1],accel[2];

void MPU6050_Init(void);
void MPU6050_Get_Pose(void);
int8_t i2cwrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int8_t i2cread(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
void get_ms(unsigned long *time);

#endif
