/*
 * stm32f4_i2c_Driver.h
 *
 *  Created on: 03/nov/2014
 *      Author: Mirko
 */

#ifndef STM32F4_I2C_DRIVER_H_
#define STM32F4_I2C_DRIVER_H_

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"

/* I2C Interrupt Priority */
#define I2C_INT_PRIORITY_DEFAULT 			0x0F
#define I2C_INT_SUBPRIORITY_DEFAULT			0x0F

/* I2C Init Options */
#define I2C_MODE_DEFAULT					I2C_Mode_I2C
#define I2C_DUTYCYCLE_DEFAULT				I2C_DutyCycle_2
#define I2C_ACK_DEFAULT						I2C_Ack_Enable
#define I2C_I2C_ACKADDRESS_DEFAULT			I2C_AcknowledgedAddress_7bit

/* Receive */
#define I2C_TIMEOUT							20000
#define I2C_RECEIVE_ERROR					-1
#define I2C_RECEIVE_OK						1

typedef enum {
	I2C_CLOCK_STANDARD = 100000,
	I2C_CLOCK_FAST_MODE = 400000,
	I2C_CLOCK_FAST_MODE_PLUS = 1000000,
	I2C_CLOCK_HIGH_SPEED = 3400000
} I2C_Speed_Type;

typedef enum {
	I2C_INTERRUPT = 1,
	I2C_POLLING = 0
} I2C_Reveice_Mode;

/* Inizializza la periferica I2C selezionata.
 * Parametri:
 * 	- I2C_TypeDef I2Cx: quale I2C della STM32F4 inizializzare.
 * 		-> possibili valori: I2C1, I2C2, I2C3
 * 			- I2C1: SCL: PB8, SDA: PB9
 * 			- I2C2: SCL: PB10, SDA: PB11
 * 			- I2C3: SCL: PH4, SDA: PH5
 *
 *	- I2C_Speed_Type I2C_Speed: velocità selezionata
 *		-> I2C_CLOCK_STANDARD = 100000
 *		   I2C_CLOCK_FAST_MODE = 400000
 *		   I2C_CLOCK_FAST_MODE_PLUS = 1000000
 *		   I2C_CLOCK_HIGH_SPEED = 3400000
 *
 *	- uint16_t I2C_OwnAddress: il proprio indirizzo.
 *
 *	- I2C_Reveice_Mode I2C_Mode: seleziona se la ricezione avviene in polling o mediante interrupt.
 *		->	I2C_INTERRUPT, I2C_POLLING
 *			nel caso sia selezionata la modalità I2C_INTERRUPT bisogna ridefinire gli handler
 *			I2Cx_EV_IRQHandler e I2Cx_ER_IRQHandler del I2C selezionato.
 */
void STM32F4_I2C_Init(I2C_TypeDef* I2Cx, I2C_Speed_Type I2C_Speed, uint16_t I2C_OwnAddress, I2C_Reveice_Mode I2C_Mode);

uint8_t STM32F4_I2C_ReadByte(I2C_TypeDef* I2Cx);

void STM32F4_I2C_WriteByte(I2C_TypeDef* I2Cx, uint8_t cAddr, uint8_t* pcBuffer);

void I2C_CleanADDRandSTOPF(I2C_TypeDef* I2Cx);

#endif /* STM32F4_I2C_DRIVER_H_ */
