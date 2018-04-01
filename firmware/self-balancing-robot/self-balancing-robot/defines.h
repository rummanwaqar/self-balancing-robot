/*
 * defines.h
 *
 * Created: 2018-03-29 7:45:51 PM
 *  Author: rumma
 */ 


#ifndef _SBR_DEFINES_H_
#define _SBR_DEFINES_H_

/*
 * Pin definitions
 */
#define F_CPU 14745600UL

// UART Config
#define USART_BAUDRATE			115200
#define UART_MAX_BUFFER_SIZE	100		// uart input char buffer size

// LEDs
#define LED_PORT		C
#define LED_RED			1
#define LED_BLUE		2
#define LED_GREEN		3

// Encoder pins
#define EN_1A_PORT		B
#define EN_1A_PIN		1
#define EN_1B_PORT		B
#define EN_1B_PIN		2
#define EN_2A_PORT		D
#define EN_2A_PIN		3
#define EN_2B_PORT		D
#define EN_2B_PIN		4
// Motor pins
#define MOTOR_PORT		D
#define MOTOR_1_PIN		5
#define MOTOR_2_PIN		6
#define MOTOR_DIR1_PORT	B
#define MOTOR_DIR1_PIN	0
#define MOTOR_DIR2_PORT	D
#define MOTOR_DIR2_PIN	7

// Encoder params
#define ENC_RATE		50.0		// speed calc at 50 Hz
#define ENC_MODE		4.0			// 4x Quadrature mode
#define ENC_COUNT_REV	700.0		// encoder counts per revolution

// Motor params
#define MOTOR_1			OCR0B
#define MOTOR_2			OCR0A
#define MOTOR_MIN_PWM	15

/*
 * MPU-6050 Settings
 */
#define MPU6050_I2CINIT 1			// init i2c in MPU6050
#define MPU6050_ADDR (0x68 <<1)		// device address - 0x68 pin low (GND), 0x69 pin high (VCC)
#define MPU6050_GETATTITUDE 	0	// fusion: DISABLED=0; MAHONY FILTER=1; DMP PROCESSOR=2
//gyro and acc scale
#define MPU6050_GYRO_FS MPU6050_GYRO_FS_1000
#define MPU6050_ACCEL_FS MPU6050_ACCEL_FS_4
// Calibration
#define MPU6050_CALIBRATEDACCGYRO 1	// set to 1 if is calibrated
#if MPU6050_CALIBRATEDACCGYRO == 1
#define MPU6050_AXOFFSET 385
#define MPU6050_AYOFFSET -453
#define MPU6050_AZOFFSET 104
#define MPU6050_GXOFFSET -32.8
#define MPU6050_GYOFFSET -7.4
#define MPU6050_GZOFFSET 14.5
#endif

/*
 * Macros definitions
 */
#define PORT_(port) PORT ## port
#define DDR_(port)  DDR  ## port
#define PIN_(port)  PIN  ## port

#define PORT(port) PORT_(port)
#define DDR(port)  DDR_(port)
#define PIN(port)  PIN_(port)

#define READ_PIN(port, pin)	((PIN(port) & _BV(pin)) >> pin)

/*
 * Custom data types
 */

#endif /* DEFINES_H_ */