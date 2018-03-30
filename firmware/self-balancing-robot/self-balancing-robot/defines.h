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
#define F_CPU 1843200UL

// UART Config
#define UART_BAUD_RATE	115200

// LEDs
#define LED_PORT		C
#define LED_RED			1
#define LED_BLUE		2
#define LED_GREEN		3

/*
 * Macros definitions
 */
#define PORT_(port) PORT ## port
#define DDR_(port)  DDR  ## port
#define PIN_(port)  PIN  ## port

#define PORT(port) PORT_(port)
#define DDR(port)  DDR_(port)
#define PIN(port)  PIN_(port)

/*
 * Custom data types
 */

#endif /* DEFINES_H_ */