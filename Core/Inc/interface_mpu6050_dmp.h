#ifndef __INTERFACE_MPU6050_DMP_H__
#define __INTERFACE_MPU6050_DMP_H__

#include "bsp_dmp.h"
#include "bsp_mpu.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

// Initialize the MPU6050: MPU6050_Init();
void MPU6050_Init(void);
// Get the pose of the MPU6050: MPU6050_Pose();
void MPU6050_Pose(void);

#endif /* __INTERFACE_MPU6050_DMP_H__ */
