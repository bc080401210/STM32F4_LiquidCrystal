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

#ifndef STM32F4_LIQUIDCRYSTAL_DRIVER_H_
#define STM32F4_LIQUIDCRYSTAL_DRIVER_H_

#include "i2c/stm32f4_i2c_Driver.h"

/* I2C Settings */
#define LC_I2Cx						I2C1
#define LC_I2C_MASTER_ADDRESS		0x00
#define LC_I2C_SLAVE_ADDRESS		0x27<<1
#define LC_I2C_SPEED				I2C_CLOCK_STANDARD

#define LC_PWRUP_DELAY				400000
#define LC_4ms_DELAY				5000
#define LC_100us_DELAY				100
#define LC_37us_DELAY				50
#define LC_DISPALY_ROW				4
#define LC_DISPALY_COL				20
#define LC_DISPLAY_LENGTH			LC_DISPALY_ROW*LC_DISPALY_COL


/* Collegamento Pin PCF8574T -> HD44780
 * P0  ->  R/S
 * P1  ->  RW
 * P2  ->  E
 * P3  ->  BackLight

 * P4-P7 -> D4-D7
 */
#define LC_BL_BIT					0x08
#define LC_EN_BIT					0x04
#define LC_RW_BIT					0x02
#define LC_RS_BIT					0x01

/* Display Instructions */
#define LC_ISTR_CLEAR				0x01
#define LC_ISTR_RETURN_HOME			0x02

/* Entry Mode Set */
#define LC_EMS						0x04
#define LC_EMS_ID					0x02		// 1 -> increment        0 -> decrement
#define LC_EMS_S					0x01		// 1 -> display shift with cursor

/* Cursor or Display Shift */
#define LC_CDS						0x10
#define LC_CDS_SC					0x08		// 1 -> select display   0 -> select cursor
#define LC_CDS_RL					0x04		// 1 -> shift right		 0 -> shift left

/* Function Set */
#define LC_FS						0x20
#define LC_FS_DL					0x10		// 1 -> 8 bit interface  0 -> 4 bit interface
#define LC_FS_N						0x08		// 1 -> 2 lines			 0 -> 1 line
#define LC_FS_F						0x04		// 1 -> 5x10 dots        0 -> 5x8 dots
#define LC_FS_DEFAULT				LC_FS | LC_FS_N

/* Display Control */
#define LC_DC						0x08
#define LC_DC_D						0x04		// display on
#define LC_DC_C						0x02		// cursor on
#define LC_DC_B						0x01		// cursor blink


/* Entry Mode Set Typedef */
typedef enum {
  INCREMENT = 1,
  DECREMENT = 0
} IncrementMode_TypeDef;

typedef enum {
  SHIFT = 1,
  NO_SHIFT = 0
} DisplayShiftMode_TypeDef;

/* Cursor or Display Shift Typedef */
typedef enum {
  CURSOR = 1,
  DISPALY = 0
} CursorDisplay_TypeDef;

typedef enum {
  RIGHT = 1,
  LEFT = 0
} ShiftDirection_TypeDef;

/* Display Control Typedef */
typedef enum {
  DISPLAY_ON = 1,
  DISPLAY_OFF = 0
} DisplayMode_TypeDef;

typedef enum {
  CURSOR_ON = 1,
  CURSOR_OFF = 0
} CursorMode_TypeDef;

typedef enum {
  BLINK_ON = 1,
  BLINK_OFF = 0
} BlinkingCursor_TypeDef;

/* Write Typedef */
typedef enum {
  DATA = 1,
  COMMAND = 0
} WriteMode_TypeDef;

typedef enum {
  DONE = 1,
  LENGTH_ERROR = 0
} WriteResult_TypeDef;

/* Set Cursor Position Typedef */
typedef enum {
  POSITION_SET = 0,
  ROW_ERROR = 1,
  COL_ERROR = 2
} CursorPositionResult_TypeDef;


/* Inizializza I2C per la comunicazione con PCF8574T utilizzando:
 * 		SDA -> PB9
 * 		SCK -> PB8
 * Di default accende il display, abilita il cursore ed il blink.
 */
void STM32F4_LiquidCrystal_Init();

/* Pulisce il display, riporta il cursore e il display (se sono stati shiftati) alla posizione
 * iniziale e azzera l'address counter della DDRAM.
 *
 * NOTA: questa funzione impone il cursore a shiftare a destra.
 */
void STM32F4_LiquidCrystal_ClearDisplay();

/* Riporta il cursore e il display alla loro posizione originale. Lascia inalterato l'address
 * counter della DDRAM.
 *
 * NOTA: Il display SOVRASCRIVE! Se si torna alla posizione originale e si scrive quando sono presenti
 * già dei caratteri, questi ultimi andranno persi. Se non si vogliono perdere i dati già scritti conviene
 * shiftare l'intero display di quanti caratteri si vuole scrivere. Lo shift dell'intero display può essere
 * fatto mediante la funzione STM32F4_LiquidCrystal_CursorDisplayShift.
 */
void STM32F4_LiquidCrystal_ReturnHome();

/* Abilita il display, il cursore ed il blinking.
 * Parametri:
 * 	- DisplayMode_TypeDef display_mode: abilita o disabilita il display.
 * 		-> possibili valori: DISPLAY_ON, DISPLAY_OFF
 * 			- DISPLAY_ON: accende il display
 * 			- DISPLAY_OFF: spegne il display
 *
 *	- CursorMode_TypeDef cursor_mode: abilita o disabilita la visualizzazione del cursore
 *		-> possibili valori: CURSOR_ON, CURSOR_OFF
 *		   	- CURSOR_ON:  abilita la visualizzazione del cursore
 *		   	- CURSOR_OFF: disabilita la visualizzazione del cursore
 *
 *	- BlinkingCursor_TypeDef blink_mode: abilita o disabilita il blinking del cursore
 *		->possibili valori: CURSOR_ON, CURSOR_OFF
 *			- BLINK_ON: abilita il blinking
 *			- BLINK_OFF: disabilita il blinking
 */
void STM32F4_LiquidCrystal_DisplayControl(DisplayMode_TypeDef display_mode, CursorMode_TypeDef cursor_mode,
		BlinkingCursor_TypeDef blink_mode);

/* Seleziona la direzione del cursore e se anche il display esegue lo shift.
 * Parametri:
 * 	- IncrementMode_TypeDef id_mode: indica la direzione di shiftamento del cursore dopo la scrittura.
 * 		-> possibili valori: INCREMENT, DECREMENT
 * 			- INCREMENT: il cursore dopo la scrittura shifta a destra
 * 			- DECREMENT: il cursore shifta a sinistra
 *
 *	- DisplayShiftMode_TypeDef ds_mode: indica se il display deve shiftare insieme al cursore dopo la
 *		scrittura. Se abilitato il cursore sembrerà sempre fermo nella posizione iniziale.
 *		-> possibili valori: SHIFT, NO_SHIFT
 *		   	- SHIFT:  abilitato a shiftare insieme al cursore
 *		   	- NO_SHIFT: non abilitato a shiftare
 */
void STM32F4_LiquidCrystal_EntryModeSet(IncrementMode_TypeDef id_mode, DisplayShiftMode_TypeDef ds_mode);

/* Seleziona se eseguire uno shift del cursore o del display e in che direzione.
 * Parametri:
 * 	- CursorDisplay_TypeDef cd: indica se shiftare il cursore o il display.
 * 		-> possibili valori: CURSOR, DISPALY
 * 			- CURSOR: seleziona il cursore
 * 			- DISPALY: seleziona il display
 *
 *	- ShiftDirection_TypeDef shift_direction: indica la direziona dello shift.
 *		-> possibili valori: RIGHT, LEFT
 *		   	- RIGHT:  lo shift sarà effettuato a destra
 *		   	- LEFT: lo shift sarà effettuato a sinistra
 *
 *	NOTA: effettuare uno shift del display non comporta la perdita dei dati già visualizzati sullo schermo.
 */
void STM32F4_LiquidCrystal_CursorDisplayShift(CursorDisplay_TypeDef cd, ShiftDirection_TypeDef shift_direction);

/* Muove il cursore nella posizione indicata.
 * Parametri:
 * 	- uint8_t row: indica la riga in cui si vuole posizionare il cursore.
 * 		-> possibili valori: 1 <= row <= 4, valori al di fuori da questo range non sortiranno alcun effetto sul
 * 			display e la funzione restituirà il valore di errore ROW_ERROR.
 *
 *	- uint8_t col: indica la colonna in cui si vuole posizionare il cursore.
 *		-> possibili valori: 1 <= row <= 20, valori al di fuori da questo range non sortiranno alcun effetto sul
 * 			display e la funzione restituirà il valore di errore COL_ERROR.
 *
 *	Return:
 * 	- POSITION_SET: operazione andata a buon fine e cursore posizionato correttamente.
 *
 * 	- ROW_ERROR: valore restituito se il parametro row è maggiore di LC_DISPLAY_ROW = 4 o minore di 0, in questo
 * 		caso è stato selezionato un valore di riga errato e la funzione non sortisce alcun effetto sul display.
 *
 * 	- COL_ERROR: valore restituito se il parametro col è maggiore di LC_DISPLAY_COL = 20 o minore di 0, in questo
 * 		caso è stato selezionato un valore di colonna errato e la funzione non sortisce alcun effetto sul display.
 */
CursorPositionResult_TypeDef STM32F4_LiquidCrystal_SetCursorPosition(uint8_t row, uint8_t col);

/* Scrive sul display.
 * Parametri:
 * 	- uint8_t* pbuff: puntatore al buffer di hex che si vuole scrivere sul display. Consultare tabella
 * 		a pag 17-18 del manuale per la corrispondenza tra hex e simbolo da visualizzare sul display.
 *
 *	- uint8_t length: lunghezza del buffer. Questo parametro non deve superare la lunghezza massima
 *		consentita pari a LC_DISPLAY_LENGTH = 80, altrimenti la funzione non sortisce alcun effetto.
 *
 * Return:
 * 	- DONE: scrittura avvenuta con successo.
 *
 * 	- LENGTH_ERROR: valore restituito se il parametro length è maggiore di LC_DISPLAY_LENGTH = 80, in
 * 		questo caso la funzione non ha sortito alcun effetto sul display.
 */
WriteResult_TypeDef STM32F4_LiquidCrystal_WriteBuffer(uint8_t* pbuff, uint8_t length);

/* Scrive sul display una stringa.
 * Parametri:
 * 	- uint8_t* pbuff: puntatore alla stringa. Non è necessario indicare la lunghezza della stringa, ma se
 * 		supera LC_DISPLAY_LENGTH = 80 i caratteri eccedenti non saranno visualizzati.
 *
 * Return:
 * 	- DONE: scrittura avvenuta con successo.
 *
 * 	- LENGTH_ERROR: valore restituito se il parametro length è maggiore di LC_DISPLAY_LENGTH = 80, in
 * 		questo caso la funzione non ha sortito alcun effetto sul display.
 */
WriteResult_TypeDef STM32F4_LiquidCrystal_WriteString(char* pbuff);


/*
----------------------------------------------------------------------
---   Funzioni Private   ---------------------------------------------
----------------------------------------------------------------------
*/
void STM32F4_LC_Delay(__IO uint32_t nTime);
void STM32F4_LC_Write(uint8_t data, WriteMode_TypeDef mode);
void STM32F4_LC_WriteCommand(uint8_t data);
void STM32F4_LC_WriteByte(uint8_t data);
void STM32F4_LC_I2C_Send(uint8_t data);
void STM32F4_LC_Write4Bit(uint8_t data, WriteMode_TypeDef mode);
void STM32F4_LC_PulseEnable(uint8_t data);

#endif /* STM32F4_LIQUIDCRYSTAL_DRIVER_H_ */
