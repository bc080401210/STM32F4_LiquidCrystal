#include "STM32F4_UART_Driver.h"


void STM32F4_USART_Init(USART_TypeDef* USARTx, uint32_t baudRate, UsartMode_TypeDef mode){
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_Inits;

	if(USARTx == USART1){
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Pin = USART1_Tx_Pin | USART1_Rx_Pin;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(USART1_GPIOx, &GPIO_InitStructure);

		GPIO_PinAFConfig(USART1_GPIOx, USART1_Tx_Pin_Source, GPIO_AF_USART1);
		GPIO_PinAFConfig(USART1_GPIOx, USART1_Rx_Pin_Source, GPIO_AF_USART1);

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	}
	else if(USARTx == USART2){
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Pin = USART2_Tx_Pin | USART2_Rx_Pin;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(USART2_GPIOx, &GPIO_InitStructure);

		GPIO_PinAFConfig(USART2_GPIOx, USART2_Tx_Pin_Source, GPIO_AF_USART2);
		GPIO_PinAFConfig(USART2_GPIOx, USART2_Rx_Pin_Source, GPIO_AF_USART2);

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	}

	USART_InitStructure.USART_BaudRate = baudRate;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;

	if(mode == INTERRUPT){
		USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
		USART_ITConfig(USARTx, USART_IT_PE, ENABLE);

		NVIC_Inits.NVIC_IRQChannel = USART2_IRQn;
		NVIC_Inits.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Inits.NVIC_IRQChannelPreemptionPriority = 0x0F;
		NVIC_Inits.NVIC_IRQChannelSubPriority = 0x0F;
		NVIC_Init(&NVIC_Inits);
	}

	USART_OverSampling8Cmd(USARTx, ENABLE);
	USART_Init(USARTx, &USART_InitStructure);
	USART_Cmd(USARTx, ENABLE);
}

void STM32F4_USART_SendSync(USART_TypeDef* USARTx, uint16_t data){
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
	USART_SendData(USARTx, data);
	while(!USART_GetFlagStatus(USARTx, USART_FLAG_TC));
}

void STM32F4_USART_SendString(USART_TypeDef* USARTx, char* pdata){

	while(*pdata){
		STM32F4_USART_SendSync(USARTx, *pdata++);
	}
}

void STM32F4_USART_SendBuff(USART_TypeDef* USARTx, uint8_t* pdata, uint32_t length){
	uint32_t i = 0;

	while(i<length){
		STM32F4_USART_SendSync(USARTx, pdata[i]);
		i++;
	}
}

uint16_t STM32F4_USART_ReadSync(USART_TypeDef* USARTx){
	while(!USART_GetFlagStatus(USARTx, USART_FLAG_RXNE));
	uint16_t data = USART_ReceiveData(USARTx);
	return data;
}
