#ifndef STM32F4_UART_DRIVER_H_
#define STM32F4_UART_DRIVER_H_

#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"

/* USART1 */
#define USART1_CTS_Pin				GPIO_Pin_11
#define USART1_RTS_Pin				GPIO_Pin_12
#define USART1_Tx_Pin				GPIO_Pin_9
#define USART1_Rx_Pin				GPIO_Pin_10
#define USART1_GPIOx				GPIOA
#define USART1_CTS_Pin_Source		GPIO_PinSource11
#define USART1_RTS_Pin_Source		GPIO_PinSource12
#define USART1_Tx_Pin_Source		GPIO_PinSource9
#define USART1_Rx_Pin_Source		GPIO_PinSource10

/* USART2 */
#define USART2_CTS_Pin				GPIO_Pin_0
#define USART2_RTS_Pin				GPIO_Pin_1
#define USART2_Tx_Pin				GPIO_Pin_2
#define USART2_Rx_Pin				GPIO_Pin_3
#define USART2_GPIOx				GPIOA
#define USART2_CTS_Pin_Source		GPIO_PinSource0
#define USART2_RTS_Pin_Source		GPIO_PinSource1
#define USART2_Tx_Pin_Source		GPIO_PinSource2
#define USART2_Rx_Pin_Source		GPIO_PinSource3

/* Write Typedef */
typedef enum {
  INTERRUPT = 1,
  POLLING = 0
} UsartMode_TypeDef;


void STM32F4_USART_Init(USART_TypeDef* USARTx, uint32_t baudRate, UsartMode_TypeDef mode);
void STM32F4_USART_SendSync(USART_TypeDef* USARTx, uint16_t data);
void STM32F4_USART_SendBuff(USART_TypeDef* USARTx, uint8_t* pdata, uint32_t length);
void STM32F4_USART_SendString(USART_TypeDef* USARTx, char* pdata);
uint16_t STM32F4_USART_ReadSync(USART_TypeDef* USARTx);

#endif /* STM32F4_UART_DRIVER_H_ */
