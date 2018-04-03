/*
 * MPU6050 
 * Author: Rumman Waqar
 * Based on library from Davide Gironi, 2012
 */

#include "defines.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/delay.h>
#include "i2cmaster.h"
#include "mpu6050.h"

// contains imu data read
volatile static Imu imu_data;
volatile static char imu_flag;

/*
 * read bytes from chip register
 */
int8_t mpu6050_readBytes(uint8_t regAddr, uint8_t length, uint8_t *data) {
	uint8_t i = 0;
	int8_t count = 0;
	if(length > 0) {
		//request register
		i2c_start(MPU6050_ADDR | I2C_WRITE);
		i2c_write(regAddr);
		_delay_us(10);
		//read data
		i2c_start(MPU6050_ADDR | I2C_READ);
		for(i=0; i<length; i++) {
			count++;
			if(i==length-1)
				data[i] = i2c_readNak();
			else
				data[i] = i2c_readAck();
		}
		i2c_stop();
	}
	return count;
}

/*
 * read 1 byte from chip register
 */
int8_t mpu6050_readByte(uint8_t regAddr, uint8_t *data) {
    return mpu6050_readBytes(regAddr, 1, data);
}

/*
 * write bytes to chip register
 */
void mpu6050_writeBytes(uint8_t regAddr, uint8_t length, uint8_t* data) {
	if(length > 0) {
		//write data
		i2c_start(MPU6050_ADDR | I2C_WRITE);
		i2c_write(regAddr); //reg
		for (uint8_t i = 0; i < length; i++) {
			i2c_write((uint8_t) data[i]);
		}
		i2c_stop();
	}
}

/*
 * write 1 byte to chip register
 */
void mpu6050_writeByte(uint8_t regAddr, uint8_t data) {
    return mpu6050_writeBytes(regAddr, 1, &data);
}

/*
 * read bits from chip register
 */
int8_t mpu6050_readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    int8_t count = 0;
    if(length > 0) {
		uint8_t b;
		if ((count = mpu6050_readByte(regAddr, &b)) != 0) {
			uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
			b &= mask;
			b >>= (bitStart - length + 1);
			*data = b;
		}
    }
    return count;
}

/*
 * read 1 bit from chip register
 */
int8_t mpu6050_readBit(uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
    uint8_t b;
    uint8_t count = mpu6050_readByte(regAddr, &b);
    *data = b & (1 << bitNum);
    return count;
}

/*
 * write bit/bits to chip register
 */
void mpu6050_writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
	if(length > 0) {
		uint8_t b = 0;
		if (mpu6050_readByte(regAddr, &b) != 0) { //get current data
			uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
			data <<= (bitStart - length + 1); // shift data into correct position
			data &= mask; // zero all non-important bits in data
			b &= ~(mask); // zero all important bits in existing byte
			b |= data; // combine data with existing byte
			mpu6050_writeByte(regAddr, b);
		}
	}
}

/*
 * write one bit to chip register
 */
void mpu6050_writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    mpu6050_readByte(regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    mpu6050_writeByte(regAddr, b);
}

/*
 * set sleep disabled
 */
void mpu6050_setSleepDisabled() {
	mpu6050_writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 0);
}

/*
 * set sleep enabled
 */
void mpu6050_setSleepEnabled() {
	mpu6050_writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 1);
}


/*
 * test connection to chip
 */
uint8_t mpu6050_testConnection() {
	uint8_t buffer[2];
	mpu6050_readBits(MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, (uint8_t *)buffer);
	if(buffer[0] == 0x34)
		return 1;
	else
		return 0;
}

/*
 * initialize the accel and gyro
 */
void mpu6050_init() {
	//init i2c
	i2c_init();
	_delay_us(10);

	//allow mpu6050 chip clocks to start up
	_delay_ms(100);

	//set sleep disabled
	mpu6050_setSleepDisabled();
	//wake up delay needed sleep disabled
	_delay_ms(10);

	//set clock source (recommended for stability)
	mpu6050_writeBits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_XGYRO);
	//set digital low pass filter bandwidth to 98Hz 
	mpu6050_writeBits(MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, MPU6050_DLPF_BW_98);
    //set sample rate 50Hz = gyro_rate (1Khz) / (1 + DIV) = 1000 / (1 + 19) => DIV = 19
	mpu6050_writeByte(MPU6050_RA_SMPLRT_DIV, 19);
	//set gyro range
	mpu6050_writeBits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS);
	//set accel range
	mpu6050_writeBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, MPU6050_ACCEL_FS);
	// enable INTA interrupt
	mpu6050_writeBits(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, 1, 1 );
	
	// enable external interrupt (INT0) 
	EICRA |= _BV(ISC01) | _BV(ISC00);	// interrupt on rising edge
	EIMSK |= _BV(INT0);					// enable int
}

/*
 * get raw data
 */
void mpu6050_getRawData(volatile Imu* imu) {
	uint8_t buffer[14];
	float accel_raw_x, accel_raw_y, accel_raw_z, gyro_raw_x, gyro_raw_y, gyro_raw_z;
	static uint32_t samples = 0;
	static float gyro_offset_x, gyro_offset_y, gyro_offset_z;
	
	mpu6050_readBytes(MPU6050_RA_ACCEL_XOUT_H, 14, (uint8_t *)buffer);

    accel_raw_x = (((int16_t)buffer[0]) << 8) | buffer[1];
    accel_raw_y = (((int16_t)buffer[2]) << 8) | buffer[3];
    accel_raw_z = (((int16_t)buffer[4]) << 8) | buffer[5];
    gyro_raw_x = (((int16_t)buffer[8]) << 8) | buffer[9];
    gyro_raw_y = (((int16_t)buffer[10]) << 8) | buffer[11];
    gyro_raw_z = (((int16_t)buffer[12]) << 8) | buffer[13];
	
	// calculate the offsets at power up
	if(samples < 64) {
		samples++;
		return;
	} else if(samples < 128) {
		gyro_offset_x += gyro_raw_x;
		gyro_offset_y += gyro_raw_y;
		gyro_offset_z += gyro_raw_z;
		samples++;
		return;
	} else if(samples == 128) {
		gyro_offset_x /= 64.0f;
		gyro_offset_y /= 64.0f;
		gyro_offset_z /= 64.0f;
		samples++;
	} else {
		gyro_raw_x -= gyro_offset_x;
		gyro_raw_y -= gyro_offset_y;
		gyro_raw_z -= gyro_offset_z;
	}
	
	// accel offsets
	#ifdef MPU6050_AXOFFSET
		accel_raw_x -= MPU6050_AXOFFSET;
	#endif
	#ifdef MPU6050_AYOFFSET
		accel_raw_y -= MPU6050_AYOFFSET;
	#endif
	#ifdef MPU6050_AZOFFSET
		accel_raw_z -= MPU6050_AZOFFSET;
	#endif
	

	// convert accelerometer readings into G's
	imu->accel.x = (float)(accel_raw_x) / MPU6050_AGAIN;
	imu->accel.y = (float)(accel_raw_y) / MPU6050_AGAIN;
	imu->accel.z = (float)(accel_raw_z) / MPU6050_AGAIN;
	
	// convert gyro readings into Radians per second
	imu->gyro.x = (float)(gyro_raw_x) / DEG2RAD(MPU6050_GGAIN);
	imu->gyro.y = (float)(gyro_raw_y) / DEG2RAD(MPU6050_GGAIN);
	imu->gyro.z = (float)(gyro_raw_z) / DEG2RAD(MPU6050_GGAIN);
}

Imu* mpu6050_getData(void)
{
	if(imu_flag == 1)
	{
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			imu_flag = 0;
			return (Imu*)&imu_data;
		}
	}
	return 0;
	
}

ISR(INT0_vect)
{
	mpu6050_getRawData(&imu_data);
	imu_flag = 1;
	PORT(LED_PORT) ^= _BV(LED_GREEN);
}