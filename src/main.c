/*
 *      Hitachi display HD44780 driver connected with a  YwRobot Arduino LCM1602 IIC V1 for STM32F407.
 *      LCM1602 incapsulates HD44780 signals in a I2C message using the display 4-bit configuration.
 *      On the other hand, the STM32 is connected with a Serial Terminal (like PuTTy or Termite).
 *      The text wrote in the terminal is send to the STM32 and printed on the display.
 *
 *      Display-STM32 Connections:
 *
 *      	SDA -> PB9
 *      	SCL -> PB8
 *      	VCC -> 5V
 *
 *      	a pull-up ~50kOmh resistor for both I2C wires is needed
 *
 *      SerialTerminal-STM Connections:
 *
 *      	Rx -> PA3
 *      	Tx -> PA2
 */


#include "stm32f4xx.h"
#include "STM32F4_LiquidCrystal_Driver.h"
#include "STM32F4_UART_Driver.h"
#include <stdio.h>


void setup();
void loop();

int main(void) {
	setup();
	while(1){
		loop();
	}
}

void setup(){
	char* c = "SETUP DONE!\n";

	STM32F4_LiquidCrystal_Init();
	STM32F4_USART_Init(USART2, 9600, INTERRUPT);

	STM32F4_USART_SendString(USART2, c);
	STM32F4_USART_SendString(USART2, "Input: \n");
}

void loop(){
}

void USART2_IRQHandler(void){
	uint16_t data = USART_ReceiveData(USART2);
	uint8_t temp = data;

	if(temp == 0x0D)
		data &= 0xFF00;

    STM32F4_LiquidCrystal_WriteString((char*)&data);
}
