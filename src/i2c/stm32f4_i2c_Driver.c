/*
 * stm32f4_i2c_Driver.c
 *
 *  Created on: 03/nov/2014
 *      Author: Mirko
 */

#include "stm32f4_i2c_Driver.h"


void STM32F4_I2C_Init(I2C_TypeDef* I2Cx,  I2C_Speed_Type I2C_Speed, uint16_t I2C_OwnAddress, I2C_Reveice_Mode I2C_Mode){

	GPIO_InitTypeDef GPIO_init;
	I2C_InitTypeDef I2C_InitStruct;

	uint32_t rcc_gpio_apb;
	uint32_t rcc_gpio_ahb;
	uint16_t gpio_pin_sda;
	uint16_t gpio_pin_scl;
	uint8_t gpio_pin_source_sda;
	uint8_t gpio_pin_source_scl;
	GPIO_TypeDef* gpio_port;
	uint8_t gpio_af_i2c;
	IRQn_Type irq_event;
	IRQn_Type irq_error;

	I2C_DeInit(I2Cx);
	I2C_SoftwareResetCmd(I2Cx, ENABLE);
	I2C_SoftwareResetCmd(I2Cx, DISABLE);

	if(I2Cx == I2C1){
		rcc_gpio_apb = RCC_APB1Periph_I2C1;
		rcc_gpio_ahb = RCC_AHB1Periph_GPIOB;

		gpio_pin_sda = GPIO_Pin_9;
		gpio_pin_scl = GPIO_Pin_8;

		gpio_port = GPIOB;
		gpio_pin_source_sda = GPIO_PinSource9;
		gpio_pin_source_scl = GPIO_PinSource8;

		gpio_af_i2c = GPIO_AF_I2C1;

		irq_event = I2C1_EV_IRQn;
		irq_error = I2C1_ER_IRQn;
	}
	else if(I2Cx == I2C2){
		rcc_gpio_apb = RCC_APB1Periph_I2C2;
		rcc_gpio_ahb = RCC_AHB1Periph_GPIOB;

		gpio_pin_sda = GPIO_Pin_11;
		gpio_pin_scl = GPIO_Pin_10;

		gpio_port = GPIOB;
		gpio_pin_source_sda = GPIO_PinSource11;
		gpio_pin_source_scl = GPIO_PinSource10;

		gpio_af_i2c = GPIO_AF_I2C2;

		irq_event = I2C2_EV_IRQn;
		irq_error = I2C2_ER_IRQn;
	}
	else if (I2Cx == I2C3){
		rcc_gpio_apb = RCC_APB1Periph_I2C3;
		rcc_gpio_ahb = RCC_AHB1Periph_GPIOH;

		gpio_pin_sda = GPIO_Pin_5;
		gpio_pin_scl = GPIO_Pin_4;

		gpio_port = GPIOH;
		gpio_pin_source_sda = GPIO_PinSource5;
		gpio_pin_source_scl = GPIO_PinSource4;

		gpio_af_i2c = GPIO_AF_I2C3;

		irq_event = I2C3_EV_IRQn;
		irq_error = I2C3_ER_IRQn;
	}

	RCC_APB1PeriphClockCmd(rcc_gpio_apb, ENABLE);
	RCC_AHB1PeriphClockCmd(rcc_gpio_ahb, ENABLE);

	//GPIO Settings
	GPIO_init.GPIO_Mode = GPIO_Mode_AF;
	GPIO_init.GPIO_Speed = GPIO_Speed_50MHz;

	//Pull up resistor
	GPIO_init.GPIO_PuPd = GPIO_PuPd_UP;

	//Open Drain
	GPIO_init.GPIO_OType = GPIO_OType_OD;
	GPIO_init.GPIO_Pin = gpio_pin_scl | gpio_pin_sda;
	GPIO_Init(gpio_port, &GPIO_init);

	GPIO_PinAFConfig(GPIOB, gpio_pin_source_scl, gpio_af_i2c);
	GPIO_PinAFConfig(GPIOB, gpio_pin_source_sda, gpio_af_i2c);
	I2C_AcknowledgeConfig(I2Cx, ENABLE);

	if(I2C_Mode == I2C_INTERRUPT){
		//NVIC Init
		NVIC_InitTypeDef NVIC_InitStructure, NVIC_InitStructure2;

		NVIC_InitStructure.NVIC_IRQChannel = irq_event;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C_INT_PRIORITY_DEFAULT;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_INT_SUBPRIORITY_DEFAULT;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		NVIC_InitStructure2.NVIC_IRQChannel = irq_error;
		NVIC_InitStructure2.NVIC_IRQChannelPreemptionPriority = I2C_INT_PRIORITY_DEFAULT;
		NVIC_InitStructure2.NVIC_IRQChannelSubPriority = I2C_INT_SUBPRIORITY_DEFAULT;
		NVIC_InitStructure2.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure2);

		I2C_ITConfig(I2Cx, I2C_IT_EVT, ENABLE);
		I2C_ITConfig(I2Cx, I2C_IT_ERR, ENABLE);
		I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);
	}

	//Init I2C
	I2C_InitStruct.I2C_ClockSpeed = I2C_Speed;
	I2C_InitStruct.I2C_Mode = I2C_MODE_DEFAULT;
	I2C_InitStruct.I2C_DutyCycle = I2C_DUTYCYCLE_DEFAULT;
	I2C_InitStruct.I2C_OwnAddress1 = I2C_OwnAddress;
	I2C_InitStruct.I2C_Ack = I2C_ACK_DEFAULT;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_I2C_ACKADDRESS_DEFAULT;
	I2C_Init(I2Cx, &I2C_InitStruct);

	I2C_StretchClockCmd(I2Cx, ENABLE);
	I2C_Cmd(I2Cx, ENABLE);
}

uint8_t STM32F4_I2C_ReadByte(I2C_TypeDef* I2Cx){
	uint8_t result = I2C_RECEIVE_OK;
	uint32_t timeout = I2C_TIMEOUT;
	uint8_t dataRx;

	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED));

	while(I2C_CheckEvent(I2Cx, I2C_EVENT_SLAVE_BYTE_RECEIVED));

	dataRx = I2C_ReceiveData(I2Cx);

	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_SLAVE_STOP_DETECTED));
	I2C_CleanADDRandSTOPF(I2Cx);
	result = I2C_RECEIVE_OK;

	return dataRx;
}

void STM32F4_I2C_WriteByte(I2C_TypeDef* I2Cx, uint8_t cAddr, uint8_t* pcBuffer){

	/* Send START condition */
	I2C_GenerateSTART(I2Cx, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send LSM303DLH address for write */
	I2C_Send7bitAddress(I2Cx, cAddr, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	/* Send the byte to be written */
	I2C_SendData(I2Cx, pcBuffer[0]);

	/* Test on EV8 and clear it */
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send STOP condition */
	I2C_GenerateSTOP(I2Cx, ENABLE);
}

void I2C_CleanADDRandSTOPF(I2C_TypeDef* I2Cx){
	while ((I2Cx->SR1 & I2C_SR1_ADDR) == I2C_SR1_ADDR){
		uint32_t temp;
		temp=I2Cx->SR1;
		temp=I2Cx->SR2;
	}
	while ((I2Cx->SR1&I2C_SR1_STOPF) == I2C_SR1_STOPF){
		uint32_t temp;
		temp=I2Cx->SR1;
		I2Cx->CR1 |= 0x1;
	}
}
