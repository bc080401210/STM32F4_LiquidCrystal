/*
 * Copyright (c) 2015, Mirko Gagliardi
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "STM32F4_LiquidCrystal_Driver.h"

void STM32F4_LiquidCrystal_Init(){
	STM32F4_I2C_Init(LC_I2Cx, LC_I2C_SPEED, LC_I2C_MASTER_ADDRESS, I2C_POLLING);

	STM32F4_LC_Delay(LC_PWRUP_DELAY);

	/* Inizializzazione e configurazione modalità 4bit, pag46 del manuale del display */
	STM32F4_LC_Write(0x03, COMMAND);
	STM32F4_LC_Delay(LC_4ms_DELAY);

	STM32F4_LC_Write(0x03, COMMAND);
	STM32F4_LC_Delay(LC_4ms_DELAY);

	STM32F4_LC_Write(0x03, COMMAND);
	STM32F4_LC_Delay(LC_100us_DELAY);

	STM32F4_LC_Write(0x02, COMMAND);
	STM32F4_LC_Delay(LC_100us_DELAY);

	/* Seleziono interfaccia a 4bit, il numero di righe e la dimensione del font. Function Set deve essere
	 * settato prima di ogni istruzione e non puo' più essere modificato.
	 */
	STM32F4_LC_WriteCommand(LC_FS_DEFAULT);

	STM32F4_LiquidCrystal_DisplayControl(DISPLAY_ON, CURSOR_ON, BLINK_ON);

	STM32F4_LiquidCrystal_EntryModeSet(INCREMENT, NO_SHIFT);


	/* TEST */
//	uint8_t pbuff[LC_DISPLAY_LENGTH];
//	uint8_t i = 0;

//	for (; i<LC_DISPLAY_LENGTH; i++){
//		pbuff[i] = 0x40 + i;
//	}

//	STM32F4_LiquidCrystal_WriteBuffer(pbuff, LC_DISPLAY_LENGTH);
	STM32F4_LC_Delay(LC_PWRUP_DELAY*5);
	STM32F4_LiquidCrystal_ClearDisplay();

	STM32F4_LiquidCrystal_WriteString("HELLO BUDDY!!");
	STM32F4_LC_Delay(LC_PWRUP_DELAY*5);
	STM32F4_LiquidCrystal_ClearDisplay();
	STM32F4_LiquidCrystal_ReturnHome();
//
//	STM32F4_LiquidCrystal_WriteString("Hola gringo!! ");
//	STM32F4_LiquidCrystal_CursorDisplayShift(DISPALY, RIGHT);
//	STM32F4_LiquidCrystal_CursorDisplayShift(DISPALY, RIGHT);
//	STM32F4_LC_Delay(LC_PWRUP_DELAY*5);
//	STM32F4_LiquidCrystal_ClearDisplay();
//
//	STM32F4_LiquidCrystal_SetCursorPosition(1,1);
}

void STM32F4_LiquidCrystal_ClearDisplay(){
	STM32F4_LC_WriteCommand(LC_ISTR_CLEAR);
	STM32F4_LC_Delay(LC_4ms_DELAY/2);
}

void STM32F4_LiquidCrystal_ReturnHome(){
	STM32F4_LC_WriteCommand(LC_ISTR_RETURN_HOME);
	STM32F4_LC_Delay(LC_4ms_DELAY/2);
}

void STM32F4_LiquidCrystal_DisplayControl(DisplayMode_TypeDef display_mode, CursorMode_TypeDef cursor_mode,
		BlinkingCursor_TypeDef blink_mode){
	uint8_t display_control = LC_DC;

	if(display_mode == DISPLAY_ON){
		display_control |= LC_DC_D;
	}

	if(cursor_mode == CURSOR_ON){
		display_control |= LC_DC_C;
	}

	if(blink_mode == BLINK_ON){
		display_control |= LC_DC_B;
	}

	STM32F4_LC_Write(display_control, COMMAND);
}

void STM32F4_LiquidCrystal_EntryModeSet(IncrementMode_TypeDef id_mode, DisplayShiftMode_TypeDef ds_mode){
	uint8_t entrymode_control = LC_EMS;

	if(id_mode == INCREMENT)
		entrymode_control |= LC_EMS_ID;

	if(ds_mode == SHIFT)
		entrymode_control |= LC_EMS_S;

	STM32F4_LC_WriteCommand(entrymode_control);
}

void STM32F4_LiquidCrystal_CursorDisplayShift(CursorDisplay_TypeDef cd, ShiftDirection_TypeDef shift_direction){
	uint8_t cd_control = LC_CDS;

	if(cd == DISPALY)
		cd_control |= LC_CDS_SC;

	if(shift_direction == RIGHT)
		cd_control |= LC_CDS_RL;

	STM32F4_LC_WriteCommand(cd_control);
}

CursorPositionResult_TypeDef STM32F4_LiquidCrystal_SetCursorPosition(uint8_t row, uint8_t col){
	CursorPositionResult_TypeDef result = ROW_ERROR;
	uint8_t cicles = 0;
	uint8_t i = 0;

	if(row > LC_DISPALY_ROW || row <= 0)
		result = ROW_ERROR;
	else if(col > LC_DISPALY_COL || col <= 0)
		result = COL_ERROR;
	else{

		if(row == 2)
			cicles = LC_DISPALY_COL * 2;
		else if(row == 3)
			cicles = LC_DISPALY_COL;
		else
			cicles = (row-1) * LC_DISPALY_COL;

		cicles += col;
		STM32F4_LiquidCrystal_ReturnHome();

		for(i = 1; i < cicles; i++)
			STM32F4_LiquidCrystal_CursorDisplayShift(CURSOR, RIGHT);

		result = POSITION_SET;
	}

	return result;
}

WriteResult_TypeDef STM32F4_LiquidCrystal_WriteBuffer(uint8_t* pbuff, uint8_t length){
	uint8_t i = 0;
	WriteResult_TypeDef result = LENGTH_ERROR;

	if(length <= LC_DISPLAY_LENGTH){

		for(; i < length; i++){
			STM32F4_LC_Write(pbuff[i], DATA);
		}

		result = DONE;
	}
	return result;
}

WriteResult_TypeDef STM32F4_LiquidCrystal_WriteString(char* pbuff){
	uint8_t count = 0;
	WriteResult_TypeDef result = LENGTH_ERROR;

	while(*pbuff && (count <= LC_DISPLAY_LENGTH)){
		STM32F4_LC_Write(*pbuff++, DATA);
		count++;
	}

	if(count <= LC_DISPLAY_LENGTH)
		result = DONE;

	return result;
}

/*
----------------------------------------------------------------------
---   Funzioni Private   ---------------------------------------------
----------------------------------------------------------------------
*/
void STM32F4_LC_WriteByte(uint8_t data){
	STM32F4_LC_Write(data, DATA);
}

void STM32F4_LC_WriteCommand(uint8_t data){
	STM32F4_LC_Write(data, COMMAND);
	STM32F4_LC_Delay(LC_37us_DELAY);
}

void STM32F4_LC_Write(uint8_t data, WriteMode_TypeDef mode){
	STM32F4_LC_Write4Bit((data & 0xF0) >> 4, mode);
	STM32F4_LC_Write4Bit(data & 0x0F, mode);
}

void STM32F4_LC_Write4Bit(uint8_t data, WriteMode_TypeDef mode){
	uint8_t temp = (data & 0x0F) << 4;

	if(mode == DATA){
		temp |= LC_RS_BIT;
	}

	temp |= LC_BL_BIT;
	STM32F4_LC_PulseEnable(temp);
}

void STM32F4_LC_PulseEnable(uint8_t data){
	STM32F4_LC_I2C_Send(data | LC_EN_BIT);
	STM32F4_LC_Delay(1);
	STM32F4_LC_I2C_Send(data & ~LC_EN_BIT);
}

void STM32F4_LC_I2C_Send(uint8_t data){
	uint8_t temp = data;

	/* Send START condition */
	I2C_GenerateSTART(LC_I2Cx, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(LC_I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send LSM303DLH address for write */
	I2C_Send7bitAddress(LC_I2Cx, LC_I2C_SLAVE_ADDRESS, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
	while(!I2C_CheckEvent(LC_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	/* Send the byte to be written */
	I2C_SendData(LC_I2Cx, temp);

	/* Test on EV8 and clear it */
	while(!I2C_CheckEvent(LC_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send STOP condition */
	I2C_GenerateSTOP(LC_I2Cx, ENABLE);
}


void STM32F4_LC_Delay(__IO uint32_t nTime){
	uint8_t i;

	while (nTime--)  // delay n us
	{
		i = 25;
		while (i--)
			; // delay 1 us
	}
}
