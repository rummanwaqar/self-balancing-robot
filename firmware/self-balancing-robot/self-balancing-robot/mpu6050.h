/*
 * MPU6050
 * Author: Rumman Waqar
 * Based on library from Davide Gironi, 2012
 */


#ifndef MPU6050_H_
#define MPU6050_H_

#include <avr/io.h>
#include "mpu6050registers.h"
#include "defines.h"

// sensitivity data
#define MPU6050_GYRO_LSB_250 131.0f
#define MPU6050_GYRO_LSB_500 65.5f
#define MPU6050_GYRO_LSB_1000 32.8f
#define MPU6050_GYRO_LSB_2000 16.4f
#define MPU6050_ACCEL_LSB_2 16384.0f
#define MPU6050_ACCEL_LSB_4 8192.0f
#define MPU6050_ACCEL_LSB_8 4096.0f
#define MPU6050_ACCEL_LSB_16 2048.0f
#if MPU6050_GYRO_FS == MPU6050_GYRO_FS_250
#define MPU6050_GGAIN MPU6050_GYRO_LSB_250
#elif MPU6050_GYRO_FS == MPU6050_GYRO_FS_500
#define MPU6050_GGAIN MPU6050_GYRO_LSB_500
#elif MPU6050_GYRO_FS == MPU6050_GYRO_FS_1000
#define MPU6050_GGAIN MPU6050_GYRO_LSB_1000
#elif MPU6050_GYRO_FS == MPU6050_GYRO_FS_2000
#define MPU6050_GGAIN MPU6050_GYRO_LSB_2000
#endif
#if MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_2
#define MPU6050_AGAIN MPU6050_ACCEL_LSB_2
#elif MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_4
#define MPU6050_AGAIN MPU6050_ACCEL_LSB_4
#elif MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_8
#define MPU6050_AGAIN MPU6050_ACCEL_LSB_8
#elif MPU6050_ACCEL_FS == MPU6050_ACCEL_FS_16
#define MPU6050_AGAIN MPU6050_ACCEL_LSB_16
#endif

//functions
extern void mpu6050_init();
extern uint8_t mpu6050_testConnection();

void mpu6050_getRawData(volatile Imu_t* imu);

Imu_t* mpu6050_getData(void);

extern void mpu6050_setSleepDisabled();
extern void mpu6050_setSleepEnabled();

extern int8_t mpu6050_readBytes(uint8_t regAddr, uint8_t length, uint8_t *data);
extern int8_t mpu6050_readByte(uint8_t regAddr, uint8_t *data);
extern void mpu6050_writeBytes(uint8_t regAddr, uint8_t length, uint8_t* data);
extern void mpu6050_writeByte(uint8_t regAddr, uint8_t data);
extern int8_t mpu6050_readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
extern int8_t mpu6050_readBit(uint8_t regAddr, uint8_t bitNum, uint8_t *data);
extern void mpu6050_writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
extern void mpu6050_writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data);
extern char imu_calibrated();

#endif
