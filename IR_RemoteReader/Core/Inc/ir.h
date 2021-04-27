/*
 * ir.h
 *
 *  Created on: Apr 27, 2021
 *      Author: kai
 */

#ifndef INC_IR_H_
#define INC_IR_H_

#include "main.h"

typedef struct
{
	uint8_t id_0:4;
	uint8_t counter_0:4;
	uint8_t id_1:4;
	uint8_t counter_1:4;
	uint8_t id_2:4;
	uint8_t counter_2:4;
	uint8_t id_3:4;
	uint8_t counter_3:4;
	uint8_t id_4:4;
	uint8_t none:4; // nur zum auffuellen auf ganzes Byte
}ir_msg_dt;

typedef struct
{
	uint32_t startBit_Pulse;
	uint32_t startBit_Pause;
	uint32_t pulse;
	uint32_t bit1_Pause;
	uint32_t bit0_Pause;

	uint8_t bitLength; // Wie viele Bit breit ist die Botschaft

	char name[10];
}ir_config_dt;

typedef enum
{
	IR_LEGACY_Ia = 0,
	IR_LEGACY_Ib,
	IR_SVX,

	IR_CONFIGS_SIZE
}ir_configs_en;

typedef struct
{
	volatile uint8_t currentBit; // Auf welches Bit schreibt er gerade?

	volatile enum states
	{
		IR_NONE = 0,
		IR_START_PULSE,
		IR_START_PAUSE,
		IR_DATA_STROBE,
		IR_DATA_PAUSE
	}state;

	ir_config_dt * config;
	ir_configs_en curCfg;
	uint8_t threshold;

	uint8_t buf[5];
	uint8_t byteCounter;
	uint8_t newMsg;

	TIM_HandleTypeDef * htim; // Timer für Flankenwechsel mit 1µs

}ir_dt;

void IR_Init(ir_dt * ir, TIM_HandleTypeDef * htim);
void ir_receive_stateMachine(ir_dt * ir, uint32_t currentDelta);

#endif /* INC_IR_H_ */
