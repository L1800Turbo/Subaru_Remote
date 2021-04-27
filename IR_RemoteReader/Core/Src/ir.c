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
		{.startBit_Pulse = 2000, .startBit_Pause = 12784, .pulse = 650, .bit1_Pause = 1800, .bit0_Pause = 845, .bitLength = 36, .name = "LEGACY a"}, /* IR_LEGACY_Ia */
		{.startBit_Pulse = 4137, .startBit_Pause =  7705, .pulse = 590, .bit1_Pause = 1530, .bit0_Pause = 590, .bitLength = 36, .name = "LEGACY b"}, /* IR_LEGACY_Ib */
		{.startBit_Pulse = 4200, .startBit_Pause =  8800, .pulse = 650, .bit1_Pause = 1650, .bit0_Pause = 555, .bitLength = 36, .name = "SVX     "}, /* IR_SVX       */
}; // TODO: zu gleiche Werte!

void IR_Init(ir_dt * ir, TIM_HandleTypeDef * htim)
{
	ir->state = IR_NONE;

	ir->currentBit = 0;
	ir->byteCounter = 0;
	memset(ir->buf, 0, 5);

	ir->newMsg = 0;

	ir->config = irConfigs;
	ir->curCfg = IR_CONFIGS_SIZE; // Als ungültig

	ir->threshold 		=    30; // %

	ir->htim = htim;
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
//				IR_Test_LOW;

			__HAL_TIM_SET_CAPTUREPOLARITY(ir->htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

			//printf("First Flag...\r\n");
			ir->state++;
			break;

		case IR_START_PULSE: // Steigende Flanke, Ende Start-Bit
//				IR_Test_HIGH;

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
			}
			else
			{
				ir->state = IR_NONE;
				printf("Start Pulse wrong (got %lu), resetting...\r\n", currentDelta);
				__HAL_TIM_SET_CAPTUREPOLARITY(ir->htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
			}
			break;

		case IR_START_PAUSE: // Fallende Flanke, Ende Start-Pause
			// TODO: Eigentlich müsste er hier auch nochmal auf die Config prüfen...
			//IR_Test_LOW;

			__HAL_TIM_SET_CAPTUREPOLARITY(ir->htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

			if(currentDelta > (ir->config[ir->curCfg].startBit_Pause - currentThreshold) && currentDelta < (ir->config[ir->curCfg].startBit_Pause + currentThreshold))
			{
				ir->currentBit = 0;
				ir->state++;
			}
			else
			{
				ir->state = IR_NONE;
				//printf("Start Pause wrong (got %lu), resetting...\r\n", currentDelta);
				__HAL_TIM_SET_CAPTUREPOLARITY(ir->htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
			}
			break;

		case IR_DATA_STROBE: // Steigende Flanke, Ende eines Strobes
			//IR_Test_HIGH;
			__HAL_TIM_SET_CAPTUREPOLARITY(ir->htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);

			if(currentDelta > (ir->config[ir->curCfg].pulse - currentThreshold) && currentDelta < (ir->config[ir->curCfg].pulse + currentThreshold))
			{
				if(ir->currentBit >= ir->config[ir->curCfg].bitLength) // Wenn alle Bits empfangen wurden zurücksetzen
				{
					ir->state = IR_NONE;
					ir->currentBit = 0;
					ir->byteCounter = 0;

					ir->newMsg = 1;
					//__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
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
				//__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
			}

			break;

		case IR_DATA_PAUSE: // Fallende Flanke nach 1- oder 0-Pause
			//IR_Test_LOW;

			__HAL_TIM_SET_CAPTUREPOLARITY(ir->htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

			if(currentDelta > (ir->config[ir->curCfg].bit0_Pause - currentThreshold) && currentDelta < (ir->config[ir->curCfg].bit0_Pause + currentThreshold))
			{
				ir->buf[ir->byteCounter] |= 0 << (/*7-*/(ir->currentBit % 8));
				//printf("0");
				ir->currentBit++;

				ir->state = IR_DATA_STROBE;
			}
			else if(currentDelta > (ir->config[ir->curCfg].bit1_Pause - currentThreshold) && currentDelta < (ir->config[ir->curCfg].bit1_Pause + currentThreshold))
			{
				ir->buf[ir->byteCounter] |= 1 << (/*7-*/(ir->currentBit % 8));
				//printf("1");
				ir->currentBit++;

				ir->state = IR_DATA_STROBE;
			}
			else
			{
				ir->state = IR_NONE;
				//printf("Wrong Pause at tbd., resetting...\r\n"); // TODO: Welches Bit?
				__HAL_TIM_SET_CAPTUREPOLARITY(ir->htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
			}

			if((ir->currentBit % 8) == 0)
			{
				ir->byteCounter++;
			}
			break;
	}
}
