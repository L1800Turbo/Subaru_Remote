/*
 * ir.h
 *
 *  Created on: Apr 27, 2021
 *      Author: kai
 */

#ifndef INC_IR_H_
#define INC_IR_H_

#include "main.h"


/**
 * Aufteilung FB-Nachricht (MSB):
 * +----------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
 * | Position |  0...3  | 4...7   | 8...11  | 12...15 | 16...19 | 20...23 | 24...27 | 28...31 | 32...36 |
 * +----------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
 * | Funktion |  ID[0]  | Cnt[0]  |  ID[1]  | Cnt[1]  |  ID[2]  | Cnt[2]  |  ID[3]  | Cnt[3]  |  ID[4]  |
 * +----------+---------+---------+---------+---------+---------+---------+---------+---------+---------+
 *
 * Aufteilung eines Buffers, passend zu dieser Struktur (MSB durch verdreht einlesen):
 * +----------+------------------+------------------+------------------+------------------+------------------+
 * | Position |       7...0      |      15...8      |     23...16      |      31...24     |     40...32      |
 * +----------+------------------+------------------+------------------+------------------+------------------+
 * | Funktion |  Cnt[0] / ID[0]  |  Cnt[1] / ID[1]  |  Cnt[2] / ID[2]  |  Cnt[3] / ID[3]  |    nnnn / ID[4]  |
 * +----------+------------------+------------------+------------------+------------------+------------------+
 *
 */
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
	uint8_t :4; // nur zum auffuellen auf ganzes Byte
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

	// Error number with last value
	struct error
	{
		enum error_values
		{
			IR_ERR_NONE = 0,
			IR_ERR_START_PULSE, /* Start pulse wrong */
			IR_ERR_START_PAUSE, /* Start pause wrong */
			IR_ERR_STROBE,		/* Strobe pulse wrong */
			IR_ERR_PAUSE		/* Pause for 1 or 0 unknown */
		}no;

		uint32_t val;
	}err;

	ir_config_dt * config;
	ir_configs_en curCfg;
	uint8_t threshold;

	uint8_t buf[5];
	uint8_t byteCounter;
	uint8_t newMsg;

	TIM_HandleTypeDef * htim; // Timer für Flankenwechsel mit 1µs

	TIM_HandleTypeDef * pwmHtim; // Timer für Sendepulse (nur Senden)

	uint32_t timeGrabber[5]; // Zeiten für alle Zeiten aufzeichnen (Debug-Zwecke)

}ir_dt;

//void IR_Init(ir_dt * ir, TIM_HandleTypeDef * htim);
void IR_Init(ir_dt * ir, TIM_HandleTypeDef * htim, TIM_HandleTypeDef * pwmHtim);
void ir_receive_stateMachine(ir_dt * ir, uint32_t currentDelta);
void ir_send_stateMachine(ir_dt * ir);

void incrementMsgCounter(ir_msg_dt * msg);

#endif /* INC_IR_H_ */
