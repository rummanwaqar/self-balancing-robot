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
#define EN_1A_PORT		D
#define EN_1A_PIN		2
#define EN_1B_PORT		C
#define EN_1B_PIN		0
#define EN_2A_PORT		D
#define EN_2A_PIN		3
#define EN_2B_PORT		D
#define EN_2B_PIN		4

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