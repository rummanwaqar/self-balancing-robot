/*
 * defines.h
 *
 * Created: 2018-03-29 7:45:51 PM
 *  Author: rumman
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
#define ENC_RATE		100.0		// speed calc at 100 Hz
#define ENC_MODE		4.0			// 4x Quadrature mode
#define ENC_COUNT_REV	700.0		// encoder counts per revolution
#define ENC_FILTER_COF	0.7			// first order IIR filter coeffiecient for motor velocity reading

// Motor params
#define MOTOR_PWM1			OCR0B
#define MOTOR_PWM2			OCR0A
#define MOTOR_MAX_RPM		130		
#define MOTOR_MAX_EFFORT	255

// Motor PID param
#define PID_I_WINDUP		500

// MPU6050 settings
#define MPU6050_ADDR		(0x68 <<1)				// device address - 0x68 pin low (GND), 0x69 pin high (VCC)
#define MPU6050_GYRO_FS		MPU6050_GYRO_FS_2000	// gyro scale
#define MPU6050_ACCEL_FS	MPU6050_ACCEL_FS_4		// accel scale
#define MPU6050_CLOCK_DIV	19						// sampling rate 50Hz = (1Khz) / (1 + DIV) = 1000 / (1 + 19) => DIV = 19
// Calibration
#define MPU6050_AXOFFSET 336
#define MPU6050_AYOFFSET -247
#define MPU6050_AZOFFSET 51


// Conversions Math
#define PI				3.14159f
#define DEG2RAD(deg)	(deg * 180.0f / PI)
#define RAD2DEG(rad)	(rad * PI / 180.0f)

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
typedef struct  
{
	float x;
	float y;
	float z;
} Vector3_t;

// imu data
typedef struct {
	Vector3_t accel;
	Vector3_t gyro;
} Imu_t;

typedef enum {
	CMD_NULL, CMD_M1, CMD_M2, CMD_M_P, CMD_M_I, CMD_M_D
} Command_t;

#endif /* DEFINES_H_ */