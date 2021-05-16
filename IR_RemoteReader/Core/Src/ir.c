/*
 * ir.c
 *
 *  Created on: Apr 27, 2021
 *      Author: kai
 */
#include "ir.h"
#include <stdio.h>
#include <string.h>

ir_config_dt irConfigs[IR_CONFIGS_SIZE] =
{
		{.startBit_Pulse = 6278, .startBit_Pause =  8567, .pulse = 650, .bit1_Pause = 1800, .bit0_Pause = 800, .bitLength = 36, .name = "LEGACY a"}, /* IR_LEGACY_Ia */
		{.startBit_Pulse = 1700, .startBit_Pause = 13100, .pulse = 660, .bit1_Pause = 1900, .bit0_Pause = 925, .bitLength = 36, .name = "LEGACY b"}, /* IR_LEGACY_Ib */
		{.startBit_Pulse = 4200, .startBit_Pause =  8800, .pulse = 650, .bit1_Pause = 1650, .bit0_Pause = 555, .bitLength = 36, .name = "SVX     "}, /* IR_SVX       */
}; // TODO: zu gleiche Werte!

void IR_Init(ir_dt * ir, TIM_HandleTypeDef * htim, TIM_HandleTypeDef * pwmHtim)
{
	ir->state = IR_NONE;

	ir->currentBit = 0;
	ir->byteCounter = 0;
	memset(ir->buf, 0, 5);

	ir->newMsg = 0;

	ir->config = irConfigs;
	ir->curCfg = IR_CONFIGS_SIZE; // Als ungültig

	ir->threshold =    30; // %

	ir->htim = htim; // Verwendeter 1µs timer

	if(pwmHtim != NULL)
	{
		ir->pwmHtim = pwmHtim;
	}
}

/**
 * Bitweise durch IR-Nachricht gehen
 *
 */
void ir_receive_stateMachine(ir_dt * ir, uint32_t currentDelta)
{
	uint32_t currentThreshold = (uint32_t) (currentDelta * (float) ir->threshold/100); // TODO: Nicht mehr in %

	switch(ir->state)
	{
		case IR_NONE: // erste fallende Flanke
			__HAL_TIM_SET_CAPTUREPOLARITY(ir->htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

			ir->state++;
			break;

		case IR_START_PULSE: // Steigende Flanke, Ende Start-Bit
			__HAL_TIM_SET_CAPTUREPOLARITY(ir->htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);

			// Nach Konfiguration suchen
			ir->curCfg = IR_CONFIGS_SIZE;
			for(ir_configs_en i=0; i<IR_CONFIGS_SIZE; i++)
			{
				if(currentDelta > (ir->config[i].startBit_Pulse - currentThreshold) && currentDelta < (ir->config[i].startBit_Pulse + currentThreshold))
				{
					ir->curCfg = i;
					break;
				}
			}

			if(ir->curCfg < IR_CONFIGS_SIZE) // Wenn er die Config gefunden hat
			{
				ir->state = IR_START_PAUSE;

				ir->timeGrabber[0] = currentDelta; // Für Debug: Zeiten aufzeichnen
			}
			else
			{
				ir->state = IR_NONE;

				ir->err.no  = IR_ERR_START_PULSE;
				ir->err.val = currentDelta;
				//printf("Start Pulse wrong (got %lu), resetting...\r\n", currentDelta); TODO Weg
				__HAL_TIM_SET_CAPTUREPOLARITY(ir->htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
			}
			break;

		case IR_START_PAUSE: // Fallende Flanke, Ende Start-Pause
			// TODO: Eigentlich müsste er hier auch nochmal auf die Config prüfen...
			__HAL_TIM_SET_CAPTUREPOLARITY(ir->htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

			if(currentDelta > (ir->config[ir->curCfg].startBit_Pause - currentThreshold) && currentDelta < (ir->config[ir->curCfg].startBit_Pause + currentThreshold))
			{
				ir->currentBit = 0;
				ir->state++;

				ir->timeGrabber[1] = currentDelta; // Für Debug: Zeiten aufzeichnen
			}
			else
			{
				ir->state = IR_NONE;

				ir->err.no  = IR_ERR_START_PAUSE;
				ir->err.val = currentDelta;
				//printf("Start Pause wrong (got %lu), resetting...\r\n", currentDelta);
				__HAL_TIM_SET_CAPTUREPOLARITY(ir->htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
			}
			break;

		case IR_DATA_STROBE: // Steigende Flanke, Ende eines Strobes
			__HAL_TIM_SET_CAPTUREPOLARITY(ir->htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);

			if(currentDelta > (ir->config[ir->curCfg].pulse - currentThreshold) && currentDelta < (ir->config[ir->curCfg].pulse + currentThreshold))
			{
				if(ir->currentBit == 0) // Für Debug: Zeiten aufzeichnen
				{
					ir->timeGrabber[2] = currentDelta;
					ir->timeGrabber[3] = 0;
					ir->timeGrabber[4] = 0;
				}
				else
				{
					ir->timeGrabber[2] = (ir->timeGrabber[2] + currentDelta) / 2;
				}

				if(ir->currentBit >= ir->config[ir->curCfg].bitLength) // Wenn alle Bits empfangen wurden zurücksetzen
				{
					ir->state = IR_NONE;
					ir->currentBit = 0;
					ir->byteCounter = 0;

					ir->newMsg = 1;
				}
				else
				{
					ir->state++;
				}

			}
			else
			{
				ir->state = IR_NONE;
				//printf("Strobe wrong, resetting...\r\n");
			}

			break;

		case IR_DATA_PAUSE: // Fallende Flanke nach 1- oder 0-Pause
			//IR_Test_LOW;

			__HAL_TIM_SET_CAPTUREPOLARITY(ir->htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

			if(currentDelta > (ir->config[ir->curCfg].bit0_Pause - currentThreshold) && currentDelta < (ir->config[ir->curCfg].bit0_Pause + currentThreshold))
			{
				//ir->buf[ir->byteCounter] |= 0 << (ir->currentBit % 8); // TODO: Eine 0 odern geht gar nicht! TESTEN
				ir->currentBit++;

				ir->state = IR_DATA_STROBE;

				if(ir->timeGrabber[3] == 0) // Für Debug: Zeiten aufzeichnen
				{
					ir->timeGrabber[3] = currentDelta;
				}
				else
				{
					ir->timeGrabber[3] = (ir->timeGrabber[3] + currentDelta) / 2;
				}
			}
			else if(currentDelta > (ir->config[ir->curCfg].bit1_Pause - currentThreshold) && currentDelta < (ir->config[ir->curCfg].bit1_Pause + currentThreshold))
			{
				ir->buf[ir->byteCounter] |= 1 << (ir->currentBit % 8);
				ir->currentBit++; // in dieser Richtung LSB

				ir->state = IR_DATA_STROBE;

				if(ir->timeGrabber[4] == 0) // Für Debug: Zeiten aufzeichnen
				{
					ir->timeGrabber[4] = currentDelta;
				}
				else
				{
					ir->timeGrabber[4] = (ir->timeGrabber[4] + currentDelta) / 2;
				}
			}
			else
			{
				ir->state = IR_NONE;
				__HAL_TIM_SET_CAPTUREPOLARITY(ir->htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
			}

			if((ir->currentBit % 8) == 0)
			{
				ir->byteCounter++;
			}
			break;
	}
}

void ir_send_stateMachine(ir_dt * ir /*, uint32_t currentDelta*/)
{
	uint8_t currentBitmask = 0;

	switch(ir->state)
	{
		case IR_NONE:
			// nichts tun...
			break;

		case IR_START_PULSE: // am Ende eines Startpulses...
			HAL_TIM_PWM_Stop(ir->pwmHtim, TIM_CHANNEL_1);

			ir->state = IR_START_PAUSE;
			__HAL_TIM_SET_COMPARE(ir->htim, TIM_CHANNEL_1, ir->config[ir->curCfg].startBit_Pause);
			break;

		case IR_START_PAUSE:
			HAL_TIM_PWM_Start(ir->pwmHtim, TIM_CHANNEL_1);

			ir->state = IR_DATA_STROBE;
			__HAL_TIM_SET_COMPARE(ir->htim, TIM_CHANNEL_1, ir->config[ir->curCfg].pulse);
			break;

		case IR_DATA_STROBE:
			HAL_TIM_PWM_Stop(ir->pwmHtim, TIM_CHANNEL_1);

			if(ir->currentBit >= ir->config[ir->curCfg].bitLength) // Wenn alle Bits empfangen wurden zurücksetzen
			{
				ir->state = IR_NONE;
				ir->currentBit = 0;
				ir->byteCounter = 0;
			}
			else
			{
				ir->state = IR_DATA_PAUSE;

				// Nächste Weite abhängig vom Bit setzen...
				currentBitmask = ir->buf[ir->byteCounter] & (1<<(ir->currentBit % 8));

				if(currentBitmask >= 1)
				{
					__HAL_TIM_SET_COMPARE(ir->htim, TIM_CHANNEL_1, ir->config[ir->curCfg].bit1_Pause);
				}
				else
				{
					__HAL_TIM_SET_COMPARE(ir->htim, TIM_CHANNEL_1, ir->config[ir->curCfg].bit0_Pause);
				}

			}
			break;

		case IR_DATA_PAUSE:
			HAL_TIM_PWM_Start(ir->pwmHtim, TIM_CHANNEL_1);

			ir->state = IR_DATA_STROBE;
			ir->currentBit++;

			if((ir->currentBit % 8) == 0)
			{
				ir->byteCounter++;
			}

			__HAL_TIM_SET_COMPARE(ir->htim, TIM_CHANNEL_1, ir->config[ir->curCfg].pulse);
			break;
	}
}

void incrementMsgCounter(ir_msg_dt * msg)
{
	for(uint8_t i=0; i<3; i++) // Die FB zählt immer in 3er-Schritten hoch TODO umbauen auf n+3%0xF, dann neu < alt prüfen
	{
		msg->counter_3 = (msg->counter_3+1) & 0xF;

		if(msg->counter_3 == 0)
		{
			msg->counter_2 = (msg->counter_2+1) & 0xF;

			if(msg->counter_2 == 0)
			{
				msg->counter_1 = (msg->counter_1+1) & 0xF;

				if(msg->counter_1 == 0)
				{
					msg->counter_0 = (msg->counter_0+1) & 0xF;
				}
			}
		}
	}
}
